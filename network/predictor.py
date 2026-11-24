"""A small learned forecaster for traffic arriving at a node.

Question it answers: how many passenger-car equivalents will reach my approach from this
neighbour in the next 20 seconds? A node needs this to decide whether to keep or start a green for
a platoon that has not arrived yet.

Model: ridge regression (a linear model with a penalty on large weights) on the seven features in
coordination.FEATURES. It is deliberately small. It trains in under a second, runs in microseconds
on a modest computer, can be inspected weight by weight, and has nothing that could behave
unpredictably. It only ever changes a score, never a safety rule.

Training data come from the simulator (train_predictor.py) or, later, from logs of the real
cameras: at each message time record the features, and 20 s later record how much actually arrived.
A model trained on the simulator learns the simulator's habits, so retrain it on real logs before
trusting it anywhere real.
"""
import json
from pathlib import Path

import numpy as np

from coordination import FEATURES, heuristic_arrivals


class Predictor:
    def __init__(self, weights=None, bias=0.0, meta=None):
        self.w = np.array(weights if weights is not None else [0.0] * len(FEATURES), dtype=float)
        self.b = float(bias)
        self.meta = meta or {}

    def predict(self, features):
        return float(np.dot(self.w, features) + self.b)

    @classmethod
    def fit(cls, X, y, ridge=1.0, meta=None):
        X, y = np.asarray(X, float), np.asarray(y, float)
        A = np.hstack([X, np.ones((len(X), 1))])
        penalty = ridge * np.eye(A.shape[1])
        penalty[-1, -1] = 0.0                                  # do not shrink the intercept
        sol = np.linalg.solve(A.T @ A + penalty, A.T @ y)
        return cls(sol[:-1], sol[-1], meta)

    def save(self, path):
        data = {"features": FEATURES, "weights": self.w.tolist(), "bias": self.b, "meta": self.meta}
        Path(path).write_text(json.dumps(data, indent=2))

    @classmethod
    def load(cls, path):
        d = json.loads(Path(path).read_text())
        if d["features"] != FEATURES:
            raise ValueError("this model was trained on a different feature list; retrain it")
        return cls(d["weights"], d["bias"], d.get("meta"))


def evaluate(model, X, y):
    """Mean absolute error of the model, of the no-learning rule, and of always guessing the mean."""
    X, y = np.asarray(X, float), np.asarray(y, float)
    pred = np.array([model.predict(f) for f in X])
    rule = np.array([heuristic_arrivals(f) for f in X])
    return {"model_mae": float(np.mean(np.abs(pred - y))),
            "rule_mae": float(np.mean(np.abs(rule - y))),
            "mean_guess_mae": float(np.mean(np.abs(y.mean() - y))),
            "samples": int(len(y))}
