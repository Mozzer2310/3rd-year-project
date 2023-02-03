#!/bin/bash
pdflatex report.tex
bibtex report.aux
pdflatex report.tex
pdflatex report.tex
shopt -s extglob
rm report.!(pdf|tex)
rm chapters/*.aux
shopt -u extglob
