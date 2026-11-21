# toymodel_v10 runs. Needs: python3, numpy; SPECTRE_DIR = clone of brentharts/spectre (for boundary_certificate/spectre_exact.py).
export SPECTRE_DIR ?= $(HOME)/repos/spectre
PY ?= python3
help:
	@echo "make quick   Lockwood merge: pair proof (Result 36, 20) + dipole tests (seconds)"
	@echo "make cavity   exact cavity completion tests (about 1 min); --scan 300 about 8 min"
	@echo "make dipole-full   exact N=24..28 and SA to Level 4 (about 10 s) -> results_dipole_py.json"
cavity:
	$(PY) t10_cavity.py --tests-only 2>&1 | tee log_cavity_tests.txt
quick:
	$(PY) t10_pair_proof.py --tests-only 2>&1 | tee log_pair_proof.txt
	$(PY) t10_dipole.py --tests-only 2>&1 | tee log_dipole_tests.txt
dipole-full:
	$(PY) t10_dipole.py --full 2>&1 | tee log_dipole_full.txt
.PHONY: help quick dipole-full cavity
