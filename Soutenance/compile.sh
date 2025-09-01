#!/bin/bash
set -euo pipefail

# Se placer dans le dossier du script
cd "$(dirname "$0")"

# Nettoyage des fichiers auxiliaires (y compris beamer)
rm -f main.{aux,bcf,bbl,blg,log,out,toc,run.xml,nav,snm,vrb,synctex.gz}
find . -type f \( -name "*.aux" -o -name "*.log" -o -name "*.out" -o -name "*.toc" -o -name "*.nav" -o -name "*.snm" -o -name "*.vrb" -o -name "*.bcf" -o -name "*.run.xml" \) -delete

# Compilation
lualatex -interaction=nonstopmode -halt-on-error main.tex
biber main
lualatex -interaction=nonstopmode -halt-on-error main.tex
lualatex -interaction=nonstopmode -halt-on-error main.tex
