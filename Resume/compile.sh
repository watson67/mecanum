#!/bin/bash
echo "=== Compilation LaTeX complète ==="
pdflatex main.tex
makeglossaries main
biber main
pdflatex main.tex
pdflatex main.tex
echo "=== Compilation terminée ==="
