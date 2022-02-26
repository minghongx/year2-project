<h1 align="center">
  ELEC222-202122 Year 2 Project<br>
  <sub><sup>Simulation of Quadruped-robot Locomotion</sup></sub>
</h1>

```mermaid
gantt
    dateFormat  YYYY-MM-DD

    Mataining this Gantt Chart [Minghong Xu] : active, 2022-02-11, 2022-03-18

    section Weekly Blogs
    Week 1 [Yang Kai, Zepeng Pang] : blog1, 2022-02-03, 7d
    Week 2 [Yang Kai, Zepeng Pang] : blog2, after blog1, 7d
    Week 3 [Zepeng Pang] : active, blog3, after blog2, 7d
    Week 4 [Zepeng Pang] : active, blog4, after blog3, 5d
    Setup GitHub Pages for publishing [Minghong Xu] : milestone, 2022-02-26, 0d
    Blog Submission : crit, after blog4, 2022-03-02
    Week 5 [Minghong Xu] : 2022-03-03, 7d

    section Project Management Documents
    Supervisor Weekly Meeting Log [Zepeng Pang] : crit, active, 2022-01-31, 2022-02-25
    LogBook is Git log: milestone, 2022-02-03, 0d
    Attendance Record [Minghong Xu] : active, 2022-02-03, 2022-02-27
    Role Allocation (responsibility matrix) [Minghong Xu] : milestone, 2022-02-14, 0d
    Contribution to Project Deliverables [TBD] : active, 2022-02-25, 2022-03-02

    section Presentation
    Poster [TBD] : crit, active, 2022-02-25, 2022-03-02
    Functional Bench Demonstration [TBD] : crit, active, 2022-02-25, 2022-03-02
    Group Talk : 2022-03-03, 8h

    section Simulation
    Setup Simulation Env : crit, setup_sim_env, 2022-02-03, 7d
    PyBullet Basic Settings & Standing :after setup_sim_env, 3d
    Research : crit, research, after setup_sim_env, 6d
    Control Each Motor Individually [Minghong Xu] : after research, 2022-02-20
    Squatting [Minghong Xu] : 2022-02-18, 2022-02-22
    Pitching [Zehao Ye, Minghong Xu] : crit, 2022-02-17, 2022-02-24
    Rolling [Kai Yang] : active, 2022-02-17, 2022-02-27
    Yawing [Zehao Ye] : active, 2022-02-17, 2022-02-26

    section Technical Report
    Pitching [Minghong Xu] : 2022-03-04, 2022-03-17
    Check Academic Style : 2022-03-10, 2d
```

## Structure of the Repository

<pre>
📦year2-proj
 ┣ 📂blogs
 ┃ ┣ 📂week1
 ┃ ┣ 📂week2
 ┃ ┣ 📂week3
 ┃ ┣ 📂week4
 ┃ ┗ 📂week5
 ┣ 📂gantt_chart
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
 ┣ 📜README.md
 ┣ 📜poetry.lock
 ┗ 📜pyproject.toml
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
