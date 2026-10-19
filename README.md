# Spectre Monotile

The Spectre Monotile project provides a simple Python tool to visualize a geometric tile pattern based on given parameters a, b, and curve_value.

## Requirements

The dependencies can usually be installed using `pip`:

```pip install numpy matplotlib opencv-python bezier```

The `bezier` package is optional: if it is not installed (it needs a compiler on some platforms), `spectre.py`
automatically falls back to `bezierpy.py`, a small pure-Python implementation with no dependencies.
`python3 bezierpy.py` runs its self test.

## Usage

To use the Spectre Monotile tool, navigate to the project directory and run:

```python3 spectre.py <a> <b> <curve_value>```  
Replace `<a>`, `<b>`, and `<curve_value>` with your desired values.

For example, to draw a spectre tile with parameters a=1, b=1, and curve_value=0.5, use:

```python3 spectre.py 1 1 0.5```

![spectre_monotile](https://github.com/Jan-Piotraschke/spectre-monotile-py/assets/78916218/706cfb54-81a1-43da-b587-6c6afe15ee08)

## Getting Help

To view a help message and learn about the available parameters and options, use:

```python3 spectre.py --help```

### Hidden Nerd Stuff

You can draw a shape or something like our spectre monotile using Fourier series.

To use an image:  
```python fourier_drawing.py your_image.png --num_components 100```

To use the Spectre Monotile shape (default parameters):  
```python fourier_drawing.py```

To specify parameters for the Spectre Monotile:  
```python fourier_drawing.py --a 1 --b 1 --curve_strength 0.5 --num_components 50```

![fourier_drawing](https://github.com/user-attachments/assets/849d0357-0caa-4a70-b838-bb69ca33907a)

### Symbolic version (SymPy → LaTeX/PDF)

`symspectre.py` computes the curved monotile exactly and symbolically in `a`, `b` and the curve strength `k`
(vertices, Bézier control points, closed-form curves, area, tangent angles) and writes a LaTeX document, compiled
to PDF if `pdflatex` is installed. The numeric values only affect the TikZ figure.

```python3 symspectre.py --a 1 --b 1 --curve_strength 0.5 --outdir out```

Use `--no-pdf` to only write the `.tex` file, and `--verify` to cross-check the symbolic curves against `spectre.py`
(needs `numpy`).
