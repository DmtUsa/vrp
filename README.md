# Vehicle Routing Problem

## Install
For installation it is recommended to use a virtual environment. This project uses 
Python 3.7.3
1. Clone or fork the repository.
2. Go into the project directory: `cd vrp`
3. Create a virtual environment `python3 -m venv venv`
4. Activate the virtual environment `source venv/bin/activate`
5. Upgrade pip `pip install --upgrade pip`
6. Install the necessary requirements: `pip install -r requirements.txt`
7. Install the project in editable mode: `pip install -e .`

## Usage

In the terminal switch to the `vrp` subdirectory and run the following command: `python3 main.py`.

This will solve the ILP for a problem and plot the solution.
The problem instance parameters as well as the random seed can be modified in `main.py`