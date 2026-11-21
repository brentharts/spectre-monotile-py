# toymodel_v9 runs. Needs: python3, numpy, scipy, sympy; for engine targets also SPECTRE_DIR and WADA_DIR.
# Usage: make help
export SPECTRE_DIR ?= $(HOME)/repos/spectre
export WADA_DIR    ?= $(HOME)/repos/wada-coudene
LEVEL ?= 11
PY ?= python3

help:
	@echo "make quick        exact Danzer checks (t9_danzer) + engine regression (minutes)"
	@echo "make patch-tests  B1c tests, level 9/10 (about 5 min)"
	@echo "make patch-long   B1c at LEVEL=$(LEVEL) (default 11: ~3.7e6 tiles, ~2 GB RAM; LEVEL=12: ~1.5e7 tiles, ~8 GB RAM; measured level 10: 865k tiles, 370 MB)"
	@echo "make win-cloud     B1d: save level-11 window clouds (3 min, 2 GB)"
	@echo "make win-tests     B1d: symmetry, explicit windows II/III, class I oracle (needs win-cloud; few min)"
	@echo "make flip-patch    B2: level-11 patch tiles (3.5 min, 2 GB) -> dflip_patch_L11.npz"
	@echo "make flip-all      B2: win-cloud, flip-patch, window table, stats, figures, tests (about 20 min)"
	@echo "make engine-stats Track A statistics, delta 0.03 (long)"
	@echo "make scan         all 60 dissection combinations at level 9 (about 6 min)"
	@echo "make all-long     patch-long then engine-stats"

quick:
	$(PY) t9_danzer.py --tests-only 2>&1 | tee log_danzer.txt
	$(PY) t9_engine.py --tests-only 2>&1 | tee log_engine_tests.txt

patch-tests:
	$(PY) t9_patch_tests.py 2>&1 | tee log_patch_tests.txt

patch-long:
	$(PY) t9_patch_long.py $(LEVEL) 2>&1 | tee log_patch_long_L$(LEVEL).txt

engine-stats:
	$(PY) t9_engine.py --stats 2>&1 | tee log_engine_stats.txt

scan:
	$(PY) t9_patch_scan.py 2>&1 | tee log_patch_scan.txt

win-cloud:
	$(PY) t9_win_cloud.py 11 2>&1 | tee log_win_cloud.txt

win-tests:
	$(PY) t9_win_tests.py 2>&1 | tee log_win_tests.txt

flip-patch:
	$(PY) t9_dflip_patch.py 11 22 2>&1 | tee log_flip_patch.txt

win-table:
	$(PY) t9_win_oracle.py --save

flip-stats:
	$(PY) t9_dflip_stats.py 2>&1 | tee log_flip_stats.txt

flip-figs:
	$(PY) t9_dflip_figs.py 2>&1 | tee log_flip_figs.txt

flip-tests:
	$(PY) t9_dflip_tests.py 2>&1 | tee log_flip_tests.txt

flip-all: win-cloud win-table flip-patch flip-stats flip-figs flip-tests

all-long: patch-long engine-stats
.PHONY: flip-patch win-table flip-stats flip-figs flip-tests flip-all win-cloud win-tests help quick patch-tests patch-long engine-stats scan all-long
