<h1 align="center">
  ELEC222-202122 Year 2 Project<br>
  <sub><sup>Simulation of Quadruped-robot Locomotion</sup></sub>
</h1>

## Structure of the Repository

<pre>
📦year2-proj
 ┣ 📂blogs
 ┃ ┣ 📂week1
 ┃ ┣ 📂week2
 ┃ ┣ 📂week3
 ┃ ┣ 📂week4
 ┃ ┗ 📂week5
 ┣ 📂poster
 ┃ ┣ 📂images
 ┃ ┣ 📂preamble
 ┃ ┃ ┣ 📜packages.tex
 ┃ ┃ ┗ 📜top_matter.tex
 ┃ ┗ 📜poster.tex
 ┣ 📂proj_mgmt_forms
 ┣ 📂report
 ┃ ┣ 📂figures
 ┃ ┣ 📂preamble
 ┃ ┃ ┣ 📜packages.tex
 ┃ ┃ ┗ 📜top_matter.tex
 ┃ ┣ 📂titlepage
 ┃ ┃ ┣ 📜LivUniCrest.pdf
 ┃ ┃ ┗ 📜titlepage.tex
 ┃ ┗ 📜report.tex
 ┣ 📂simulation
 ┣ ┗ 📂refs
 ┣ 📜.editorconfig
 ┣ 📜.gitignore
 ┣ 📜gantt_chart.ipynb
 ┣ 📜poetry.lock
 ┣ 📜pyproject.toml
 ┗ 📜README.md
</pre>

Folder / File | Description
:---: | :---
📜.editorconfig | EditorConfig helps maintain consistent coding styles for multiple developers working on the same project across various editors and IDEs. The EditorConfig project consists of a file format for defining coding styles and a collection of text editor plugins that enable editors to read the file format and adhere to defined styles.
📂preamble | Where preliminary or preparatory statements are placed.
📜packages.tex | Where package-importing statements are placed.
📜top_matter.tex | Where metadata about a LaTeX document is placed.
📂simulation | Where simulation code is placed.
📜pyproject.toml | pyproject.toml contains build system requirements and [core metadata](https://packaging.python.org/en/latest/specifications/core-metadata/) of Python projects. *python-poetry* is a reference implementation for using pyproject.toml.
📜poetry.lock | This file prevents from automatically getting the latest versions of dependencies. Running `poetry install` when a poetry.lock file is present resolves and installs all dependencies that are listed in pyproject.toml, but Poetry uses the exact versions listed in poetry.lock to ensure that the package versions are consistent for everyone working on a project. To update to the latest versions, use the `poetry update` command. This will fetch the latest matching versions (as per pyproject.toml) and update the lock file with the new versions. (This is equivalent to deleting the poetry.lock file and running `poetry install` again.)
