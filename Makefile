# ------------------------------------------------------ curved spectre monotile
#
#   make               list the targets (same as `make help`)
#   make test          self tests: bezierpy.py and spectre_operator.py
#   make verify        check the symbolic (SymPy) curves against spectre.py
#   make check         test + verify
#   make figures       rebuild figures/spectre_operator_variations.png
#   make pdf           build/symspectre.pdf (symbolic LaTeX document, needs pdflatex)
#   make show          open the interactive matplotlib window
#   make all           check + figures + pdf
#   make install       pip install -r requirements.txt
#   make install-min   pip install only numpy, matplotlib and sympy
#   make clean         remove build/ and Python caches
#
# The `bezier` package is optional: without it spectre.py falls back to the
# pure-Python bezierpy.py, so `make install-min` is enough for everything here.

PYTHON ?= python3

FIGDIR := figures
BUILD  := build
VARIATIONS_PNG := $(FIGDIR)/spectre_operator_variations.png

.DEFAULT_GOAL := help
.PHONY: help all check test verify figures pdf show install install-min clean

help:  ## list the targets
	@grep -hE '^[a-zA-Z_-]+:.*## ' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*## "} {printf "  make %-12s %s\n", $$1, $$2}'

all: check figures pdf  ## check + figures + pdf

check: test verify  ## test + verify

test:  ## run the self tests (bezierpy, spectre_operator)
	$(PYTHON) bezierpy.py
	$(PYTHON) spectre_operator.py --tests-only

verify:  ## symbolic (SymPy) curves vs the numeric spectre.py
	$(PYTHON) symspectre.py --verify --no-pdf --outdir $(BUILD)

figures:  ## rebuild figures/spectre_operator_variations.png
	$(PYTHON) spectre_operator.py --skip-tests --no-show --save $(VARIATIONS_PNG)

pdf:  ## build/symspectre.pdf, the symbolic LaTeX document (needs pdflatex)
	$(PYTHON) symspectre.py --outdir $(BUILD)

show:  ## run the tests, then open the interactive variations window
	$(PYTHON) spectre_operator.py

install:  ## pip install -r requirements.txt
	$(PYTHON) -m pip install -r requirements.txt

install-min:  ## pip install numpy matplotlib sympy (bezier not needed)
	$(PYTHON) -m pip install numpy matplotlib sympy

clean:  ## remove build/ and Python caches
	rm -rf $(BUILD) __pycache__
