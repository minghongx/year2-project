<h1 align="center">
  Year 2 Project | ELEC222<br>
  <sub><sup>Simulating postural control on a quadruped robot in PyBullet</sup></sub>
</h1>

> [!IMPORTANT]
> We discuss various cases because the coordinate systems are asymmetric relative to the body centre.

## Yawing

<h3 align="center">
  <details>
    <summary>Diagram</summary>
    <p align="center">
      <img width="70%" src="report/figures/coordinate_transformations_in_yawing.jpg">
    </p>
  </details>
</h3>

### Right foreleg

$$
\begin{bmatrix}
   x_\text{after yawing} \\
   y_\text{after yawing} \\
   1                     \\
\end{bmatrix}
\=
\begin{bmatrix}
   1 & 0 & -L \\
   0 & 1 & -W \\
   0 & 0 &  1 \\
\end{bmatrix}
\begin{bmatrix}
   \cos\beta & -\sin\beta & 0 \\
   \sin\beta &  \cos\beta & 0 \\
   0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
   1 & 0 & L \\
   0 & 1 & W \\
   0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
   x \\
   y \\
   1 \\
\end{bmatrix}
$$

### Left foreleg

$$
\begin{bmatrix}
   x_\text{after yawing} \\
   y_\text{after yawing} \\
   1                     \\
\end{bmatrix}
\=
\begin{bmatrix}
   1 & 0 & -L \\
   0 & 1 &  W \\
   0 & 0 &  1 \\
\end{bmatrix}
\begin{bmatrix}
   \cos\beta & -\sin\beta & 0 \\
   \sin\beta &  \cos\beta & 0 \\
   0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
   1 & 0 &  L \\
   0 & 1 & -W \\
   0 & 0 &  1 \\
\end{bmatrix}
\begin{bmatrix}
   x \\
   y \\
   1 \\
\end{bmatrix}
$$

### Right hind leg

$$
\begin{bmatrix}
   x_\text{after yawing} \\
   y_\text{after yawing} \\
   1                     \\
\end{bmatrix}
\=
\begin{bmatrix}
   1 & 0 &  L \\
   0 & 1 & -W \\
   0 & 0 &  1 \\
\end{bmatrix}
\begin{bmatrix}
   \cos\beta & -\sin\beta & 0 \\
   \sin\beta &  \cos\beta & 0 \\
   0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
   1 & 0 & -L \\
   0 & 1 &  W \\
   0 & 0 &  1 \\
\end{bmatrix}
\begin{bmatrix}
   x \\
   y \\
   1 \\
\end{bmatrix}
$$

### Left hind leg

$$
\begin{bmatrix}
   x_\text{after yawing} \\
   y_\text{after yawing} \\
   1                     \\
\end{bmatrix}
\=
\begin{bmatrix}
   1 & 0 & L \\
   0 & 1 & W \\
   0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
   \cos\beta & -\sin\beta & 0 \\
   \sin\beta &  \cos\beta & 0 \\
   0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
   1 & 0 & -L \\
   0 & 1 & -W \\
   0 & 0 &  1 \\
\end{bmatrix}
\begin{bmatrix}
   x \\
   y \\
   1 \\
\end{bmatrix}
$$


## Pitching

<h3 align="center">
  <details>
    <summary>Diagram</summary>
    <p align="center">
      <img width="70%" src="report/figures/coordinate_transformations_in_pitching.jpg">
    </p>
  </details>
</h3>

### Front Legs

$$
\begin{bmatrix}
   x_\text{after pitching} \\
   z_\text{after pitching} \\
   1                       \\
\end{bmatrix}
\=
\begin{bmatrix}
   1 & 0 & -L \\
   0 & 1 &  0 \\
   0 & 0 &  1 \\
\end{bmatrix}
\begin{bmatrix}
   \cos\delta & -\sin\delta &  0 \\
   \sin\delta &  \cos\delta &  0 \\
   0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
   1 & 0 & L \\
   0 & 1 & 0 \\
   0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
   x \\
   z \\
   1 \\
\end{bmatrix}
$$

### Hind Legs

$$
\begin{bmatrix}
   x_\text{after pitching} \\
   z_\text{after pitching} \\
   1                       \\
\end{bmatrix}
\=
\begin{bmatrix}
   1 & 0 & L \\
   0 & 1 & 0 \\
   0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
   \cos\delta & -\sin\delta & 0 \\
   \sin\delta &  \cos\delta & 0 \\
   0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
   1 & 0 & -L \\
   0 & 1 &  0 \\
   0 & 0 &  1 \\
\end{bmatrix}
\begin{bmatrix}
   x \\
   z \\
   1 \\
\end{bmatrix}
$$


## Rolling

<h3 align="center">
  <details>
    <summary>Diagram</summary>
    <p align="center">
      <img width="50%" src="report/figures/coordinate_transformations_in_rolling.jpg">
    </p>
  </details>
</h3>

### Right Legs

$$
\begin{bmatrix}
   y_\text{after rolling} \\
   z_\text{after rolling} \\
   1                      \\
\end{bmatrix}
\=
\begin{bmatrix}
   1 & 0 & -W \\
   0 & 1 &  0 \\
   0 & 0 &  1 \\
\end{bmatrix}
\begin{bmatrix}
   \cos\lambda & -\sin\lambda & 0 \\
   \sin\lambda &  \cos\lambda & 0 \\
   0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
   1 & 0 & W \\
   0 & 1 & 0 \\
   0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
   y \\
   z \\
   1 \\
\end{bmatrix}
$$

### Left Legs

$$
\begin{bmatrix}
   y_\text{after rolling} \\
   z_\text{after rolling} \\
   1                      \\
\end{bmatrix}
\=
\begin{bmatrix}
   1 & 0 & W \\
   0 & 1 & 0 \\
   0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
   \cos\lambda & -\sin\lambda & 0 \\
   \sin\lambda &  \cos\lambda & 0 \\
   0 & 0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
   1 & 0 & -W \\
   0 & 1 &  0 \\
   0 & 0 &  1 \\
\end{bmatrix}
\begin{bmatrix}
   y \\
   z \\
   1 \\
\end{bmatrix}
$$


## Forward Kinematics

Forward kinematics is a mapping function from joint space to Cartesian space.


## Inverse Kinematics

Inverse kinematics is an inverse function of the forward kinematics (from Cartesian space to joint space).


## Structure of the Repository

<pre>
📦year2-project
 ┣ 📂.vscode
 ┣ 📂docs
 ┣ 📂feedback
 ┣ 📂hexo
 ┃ ┣ 📂themes/next
 ┃ ┃ ┗ 📜_config.yml
 ┃ ┗ 📜_config.yml
 ┣ 📂poster
 ┣ 📂proj_mgmt_forms
 ┣ 📂refs
 ┣ 📂report
 ┃ ┣ 📂preamble
 ┃ ┃ ┣ 📜packages.tex
 ┃ ┃ ┗ 📜top_matter.tex
 ┃ ┣ 📂front_matter
 ┃ ┣ 📂body
 ┃ ┣ 📂back_matter
 ┃ ┣ 📂figures
 ┃ ┣ 📜.latexmkrc
 ┃ ┣ 📜refs.bib
 ┃ ┣ 📜report.tex
 ┃ ┗ 📜snippets.tex
 ┣ 📂sde_report
 ┣ 📂simulation
 ┣ 📜.editorconfig
 ┣ 📜.gitattributes
 ┣ 📜.gitignore
 ┣ 📜LICENSE
 ┣ 📜poetry.lock
 ┣ 📜pyproject.toml
 ┗ 📜README.md
</pre>

Folder / File | Description
:--- | :---
📂docs | From where GitHub Pages site is built.
📂hexo | Hexo manages the project website here and publishes website to 📂docs.
📜.editorconfig | EditorConfig helps maintain consistent coding styles for multiple developers working on the same project across various editors and IDEs. The EditorConfig project consists of a file format for defining coding styles and a collection of text editor plugins that enable editors to read the file format and adhere to defined styles.
📂preamble | Where preliminary or preparatory statements are placed.
📜packages.tex | Where package-importing statements are placed.
📜top_matter.tex | Where metadata about a LaTeX document is placed.
📂simulation | Where simulation code is placed.
📜pyproject.toml | pyproject.toml contains build system requirements and [core metadata](https://packaging.python.org/en/latest/specifications/core-metadata/) of Python projects. *python-poetry* is a reference implementation for using pyproject.toml.
📜poetry.lock | This file prevents from automatically getting the latest versions of dependencies. Running `poetry install` when a poetry.lock file is present resolves and installs all dependencies that are listed in pyproject.toml, but Poetry uses the exact versions listed in poetry.lock to ensure that the package versions are consistent for everyone working on a project. To update to the latest versions, use the `poetry update` command. This will fetch the latest matching versions (as per pyproject.toml) and update the lock file with the new versions. (This is equivalent to deleting the poetry.lock file and running `poetry install` again.)


## Feedback

![feedback of bench inspection from supervisor](feedback/bench_inspection/supervisor.pdf.jpg)
![feedback of bench inspection from assessor](feedback/bench_inspection/assessor.pdf.jpg)
![feedback of bench inspection from supervisor](feedback/proj_report/supervisor.pdf.jpg)


## Activity Diary

```mermaid
gantt
    dateFormat  YYYY-MM-DD

    Mataining this Gantt Chart [Minghong Xu] : 2022-02-11, 2022-03-18

    section Weekly Blogs
    Week 1 [Yang Kai, Zepeng Pang] : blog1, 2022-02-03, 7d
    Week 2 [Yang Kai, Zepeng Pang] : blog2, after blog1, 7d
    Week 3 [Zepeng Pang] : blog3, after blog2, 7d
    Week 4 [Zepeng Pang] : blog4, after blog3, 5d
    Setup GitHub Pages for publishing [Minghong Xu] : milestone, 2022-02-26, 0d
    Blog Submission : crit, after blog4, 2022-03-02

    section Project Management Documents
    Supervisor Weekly Meeting Log [Zepeng Pang] : crit, 2022-01-31, 2022-02-25
    LogBook is Git log: milestone, 2022-02-03, 0d
    Attendance Record [Minghong Xu, Zepeng Pang] : 2022-02-03, 2022-02-27
    Role Allocation (responsibility matrix) [Minghong Xu] : milestone, 2022-02-14, 0d
    Contribution to Project Deliverables [Zepeng Pang] : 2022-02-25, 2022-03-02

    section Presentation
    Poster [Zehao Ye, Kai Yang] : crit, 2022-02-25, 2022-03-02
    Functional Bench Demonstration [Zepeng Pang] : crit, 2022-02-25, 2022-03-02
    Group Talk : 2022-03-03, 8h

    section Simulation
    Setup Simulation Env : crit, setup_sim_env, 2022-02-03, 7d
    PyBullet Basic Settings & Standing :after setup_sim_env, 3d
    Research : crit, research, after setup_sim_env, 6d
    Control Each Motor Individually [Minghong Xu] : after research, 2022-02-20
    Squatting [Minghong Xu] : 2022-02-18, 2022-02-22
    Pitching [Zehao Ye, Minghong Xu] : crit, 2022-02-17, 2022-02-24
    Rolling [Kai Yang, Minghong Xu] : 2022-02-17, 2022-02-27
    Yawing [Zehao Ye, Minghong Xu] : 2022-02-17, 2022-02-26

    section Sustainability
    Found a good reference book [Kai Yang] : milestone, 2022-02-25, 0d
    Sustainability, DevelopmentReport, and Ethical Evaluation Report [Kai Yang] : 2022-02-26, 2022-03-07

    section Project Report
    Introduction [Zepeng Pang] : 2022-03-04, 2022-03-17
    Materials and Methods [Minghong Xu] : 2022-03-04, 2022-03-17
    Results and Analysis [Zehao Ye] : 2022-03-04, 2022-03-17
    Discussion and Conclusions [Zepeng Pang, Zehao Ye, Kai Yang] : 2022-03-04, 2022-03-17
    Appendices [Zepeng Pang, Minghong Xu]: 2022-03-17, 2022-03-18
    Abstract [Zepeng Pang] : 2022-03-17, 2022-03-18
    Check Academic Style : 2022-03-16, 2d
    Submission DDL: milestone, 2022-03-18, 0d
```
