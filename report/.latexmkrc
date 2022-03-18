$pdf_mode = 5;  # 5 represents using $xelatex

# Specifies the command line for the LaTeX processing program of when the xelatex program is called for.
$xelatex = "xelatex -synctex=1 -interaction=nonstopmode -file-line-error -shell-escape %O %S";

# The BibTeX processing program
$bibtex =  "bibtex %O %S";

# When the source file uses bbl files for bibliography, run bibtex or biber as needed to regenerate the bbl files
$bibtex_use = 2;

$out_dir = "build";

@default_files = ("report.tex");
@default_excluded_files = ();
