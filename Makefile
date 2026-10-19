# ------------------------------------------------------ curved spectre monotile
#
#   make               list the targets (same as `make help`)
#   make test          self tests: bezierpy.py, spectre_operator.py, spectre_fields.py, spectre_fem.py
#   make verify        check the symbolic (SymPy) curves against spectre.py
#   make check         test + verify
#   make figures       rebuild every figure in figures/
#   make pdf           build/symspectre.pdf (symbolic LaTeX document, needs pdflatex)
#   make show          open the interactive per-edge variations window
#   make show-fields   open the interactive distance-field window
#   make show-fem      open the interactive finite-element window (torsion, modes, heat)
#   make all           check + figures + pdf
#   make install       pip install -r requirements.txt
#   make install-min   pip install only numpy, scipy, matplotlib and sympy
#   make clean         remove build/ and Python caches
#
# The `bezier` package is optional: without it spectre.py falls back to the
# pure-Python bezierpy.py, so `make install-min` is enough for everything here.

PYTHON ?= python3

FIGDIR := figures
BUILD  := build
VARIATIONS_PNG := $(FIGDIR)/spectre_operator_variations.png
FIELDS_PNG     := $(FIGDIR)/spectre_distance_fields.png
FEM_PNG        := $(FIGDIR)/spectre_fem_fields.png

.DEFAULT_GOAL := help
.PHONY: help all check test verify figures pdf show show-fields show-fem install install-min clean

help:  ## list the targets
	@grep -hE '^[a-zA-Z_-]+:.*## ' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*## "} {printf "  make %-12s %s\n", $$1, $$2}'

all: check figures pdf  ## check + figures + pdf

check: test verify  ## test + verify

test:  ## run the self tests (bezierpy, spectre_operator, spectre_fields, spectre_fem)
	$(PYTHON) bezierpy.py
	$(PYTHON) spectre_operator.py --tests-only
	$(PYTHON) spectre_fields.py --tests-only
	$(PYTHON) spectre_fem.py --tests-only

verify:  ## symbolic (SymPy) curves vs the numeric spectre.py
	$(PYTHON) symspectre.py --verify --no-pdf --outdir $(BUILD)

figures:  ## rebuild every figure in figures/
	$(PYTHON) spectre_operator.py --skip-tests --no-show --save $(VARIATIONS_PNG)
	$(PYTHON) spectre_fields.py --skip-tests --no-show --save $(FIELDS_PNG)
	$(PYTHON) spectre_fem.py --skip-tests --no-show --save $(FEM_PNG)

pdf:  ## build/symspectre.pdf, the symbolic LaTeX document (needs pdflatex)
	$(PYTHON) symspectre.py --outdir $(BUILD)

show:  ## run the tests, then open the interactive per-edge variations window
	$(PYTHON) spectre_operator.py

show-fields:  ## run the tests, then open the interactive distance-field window
	$(PYTHON) spectre_fields.py

show-fem:  ## run the tests, then open the interactive finite-element window
	$(PYTHON) spectre_fem.py

install:  ## pip install -r requirements.txt
	$(PYTHON) -m pip install -r requirements.txt

install-min:  ## pip install numpy scipy matplotlib sympy (bezier not needed)
	$(PYTHON) -m pip install numpy scipy matplotlib sympy

clean:  ## remove build/ and Python caches
	rm -rf $(BUILD) __pycache__
