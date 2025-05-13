import numpy as np
import staliro
import staliro.models
import staliro.optimizers
import staliro.specifications

from RoverSim import SimulationModule


@staliro.models.model()
def model(sample: staliro.Sample) -> staliro.Trace[list[float]]:
    sim = SimulationModule()
    CPV = 6  # or any appropriate value
    t_s = 0.1  # time step, for example
    sim.pos = [sample.static["x"], sample.static["y"]]  # type: ignore
    states, wheels, g, robot, a, b, w, r, epsilon, m, J, kf, CM, h, size_body, c1, c2, c3, c4, T, dt, t, DO_PLOTS, RUN_ANIMATION, SPEED = sim.load_parameters(CPV, t_s)
    states, wheels, g, robot, a, b, w, r, epsilon, m, J, kf, CM, h, size_body, c1, c2, c3, c4, T, dt, t, DO_PLOTS, RUN_ANIMATION, SPEED = sim.run_simulator(states, wheels, g, robot, a, b, w, r, epsilon, m, J, kf, CM, h, size_body, c1, c2, c3, c4, T, dt, t, CPV, DO_PLOTS=0, RUN_ANIMATION=0, SPEED=SPEED)

    return staliro.Trace(times=t, states=states.T)


if __name__ == "__main__":
    opts = staliro.TestOptions(
        iterations=5,
        static_inputs={
            "x": (0, 20),
            "y": (-20, 0),
        }
    )
    req = "always (psi >= 0.0 and psi <= 70.0)"
    spec = staliro.specifications.rtamt.parse_dense(req, {"psi": 2})
    opt = staliro.optimizers.DualAnnealing()
    res = staliro.test(model, spec, opt, opts)
    best = min((eval for run in res for eval in run.evaluations), key=lambda e: e.cost)

    states = np.array(best.extra.trace.states).transpose()
    sim = SimulationModule()
    sim.animate_2d_noflip(states)
