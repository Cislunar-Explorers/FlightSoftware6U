#!/usr/bin/env python

"""The setup script."""

from setuptools import find_packages, setup

with open("README.rst") as readme_file:
    readme = readme_file.read()

with open("HISTORY.rst") as history_file:
    history = history_file.read()

includes = ["todo_project_name", "todo_project_name.*"]

requirements = ["numpy", "python-dotenv", "sqlalchemy"]

dev_req = ["black", "flake8", "pip", "pre-commit", "sphinx", "watchdog"]

docs_req = ["sphinx", "sphinx-rtd-theme"]

test_req = ["coverage", "pytest"]

release_req = ["bump2version", "wheel", "twine"]

extra_req = {"dev": dev_req + docs_req + test_req + release_req}

setup(
    author="Todo Full Name",
    author_email="todo@email.com",
    python_requires=">=3.6",
    classifiers=[
        "Development Status :: 2 - Pre-Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Natural Language :: English",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
    ],
    description="Todo short description",
    extras_require=extra_req,
    install_requires=requirements,
    license="MIT license",
    long_description=readme + "\n\n" + history,
    include_package_data=True,
    keywords="todo_project_name",
    name="todo_project_name",
    packages=find_packages(include=includes),
    url="https://github.com/todo_github_username/todo_project_name",
    version="0.1.0",
    zip_safe=False,
)
