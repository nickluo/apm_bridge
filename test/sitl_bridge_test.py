import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from sitl_motor_calibration import fit_samples, read_samples, write_parameters
from sitl_support import MotorModel, VerticalEstimator, enu_flu, euler, projected_force


ROOT = Path(__file__).resolve().parents[1]


def q(x=0.0, y=0.0, z=0.0, w=1.0):
    return SimpleNamespace(x_val=x, y_val=y, z_val=z, w_val=w)


def test_frames_and_specific_force():
    assert euler(enu_flu(q())) == pytest.approx((0, 0, math.pi / 2))
    assert euler(enu_flu(q(z=math.sqrt(.5), w=math.sqrt(.5)))) == pytest.approx((0, 0, 0))
    k = SimpleNamespace(orientation=q(), linear_acceleration=SimpleNamespace(x_val=0, y_val=0, z_val=-3))
    assert projected_force(k, 2.0, 9.80665) == pytest.approx(25.6133)
    # A tilted stationary body does not support weight with its full body-Z thrust.
    k.orientation = q(y=math.sin(math.pi / 6), w=math.cos(math.pi / 6))
    k.linear_acceleration.z_val = 0
    assert projected_force(k, 2.0, 9.80665) == pytest.approx(9.80665)


def test_default_parameters_and_model():
    config = yaml.safe_load((ROOT / "parameters/sitl_parameters.yaml").read_text())
    params = config["custom_link_bridge"]["ros__parameters"]
    assert params["mass"] == 2.0
    assert not params["voltage_compensation"] and not params["baro_compensation"]
    assert params["gimbal_port"] == "" and params["airsim"]["enabled"]
    motor = params["motor_parameters"]
    model = MotorModel(**motor)
    assert model.throttle(2 * 9.80665) == pytest.approx(0.16976126, abs=.005)
    assert model.throttle(0) == 0
    assert model.throttle(1000) == 0.5
    with pytest.raises(ValueError):
        model.throttle(float("nan"))


def test_fit_and_output_shape(tmp_path):
    force = np.linspace(10, 45, 30)
    samples = list(zip(force, 0.01162 * force - 0.0582))
    model, report = fit_samples(samples, 2.0, 9.80665, 1, .5, .01)
    assert model.C == pytest.approx(.01162)
    assert model.D == pytest.approx(-.0582)
    assert report["throttle_max_error"] < 1e-12
    output = tmp_path / "fitted.yaml"
    write_parameters(ROOT / "parameters/sitl_parameters.yaml", output, model, 2.0, 9.80665)
    p = yaml.safe_load(output.read_text())["custom_link_bridge"]["ros__parameters"]
    assert p["motor_parameters"]["n"] == 1
    assert p["motor_parameters"]["vbat_a"] == 0
    assert p["airsim"]["enabled"] and p["mass"] == 2.0
    for bad in ([(float("nan"), .1)] * 5, [(20, .1)] * 5, [(20, -.1)] * 5):
        with pytest.raises(ValueError):
            fit_samples(bad, 2.0, 9.80665, 1, .5, .01)


def test_cubic_fit_and_residual_rejection():
    force = np.linspace(8, 30, 40)
    throttle = 2e-6 * force ** 3 - .00012 * force ** 2 + .018 * force - .04
    model, report = fit_samples(list(zip(force, throttle)), 2, 9.80665, 3, .5, .01)
    assert (model.A, model.B, model.C, model.D) == pytest.approx((2e-6, -.00012, .018, -.04))
    assert report["throttle_max_error"] < 1e-12
    assert model.thrust_limit == pytest.approx(float(throttle.max()))
    throttle[10] += .05
    with pytest.raises(ValueError, match="residual"):
        fit_samples(list(zip(force, throttle)), 2, 9.80665, 3, .5, .01)


def test_vertical_estimator_filters_stationary_baro_noise():
    estimator = VerticalEstimator()
    estimator.update(123.4)
    noise = np.random.default_rng(1234)
    for index in range(1000):
        estimator.predict(0.0, .005)
        if index % 2 == 0:
            estimator.update(123.4 + noise.normal(0, .35))
    assert abs(estimator.height - 123.4) < .15
    assert abs(estimator.velocity) < .3
    with pytest.raises(ValueError):
        estimator.predict(float("nan"), .005)
    with pytest.raises(ValueError):
        estimator.update(float("nan"))


def test_vertical_estimator_tracks_accelerating_flight():
    estimator = VerticalEstimator()
    estimator.update(100.0)
    noise = np.random.default_rng(5678)
    for index in range(1, 1001):
        t = index * .005
        estimator.predict(1.0, .005)
        if index % 2 == 0:
            estimator.update(100 + .5 * t * t + noise.normal(0, .35))
    assert estimator.height == pytest.approx(112.5, abs=.2)
    assert estimator.velocity == pytest.approx(5.0, abs=.2)


def test_measured_motor_calibration_matches_parameters():
    samples = read_samples(ROOT / "parameters/sitl_motor_calibration.csv")
    model, report = fit_samples(samples, 2.0, 9.80665, 1, .5, .01)
    params = yaml.safe_load((ROOT / "parameters/sitl_parameters.yaml").read_text())
    configured = params["custom_link_bridge"]["ros__parameters"]["motor_parameters"]
    assert len(samples) == 42
    for key in ("A", "B", "C", "D"):
        assert configured[key] == pytest.approx(getattr(model, key), rel=1e-10, abs=1e-12)
    assert report["throttle_max_error"] < .002
    assert report["r_squared"] > .9999
