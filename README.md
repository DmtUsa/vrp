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

In the terminal switch to the `shift_planning` directory and run the following command: `python3 main.py`.

This will solve the ILP for the original problem instance and run GA with default parameters and `random seed = 3`.
The default parameters of GA and a random seed can be modified in `main.py`

To run ILP and GA for another problem instance,
save the corresponding input files in the `shift_planning/input_data/plan_<PROBLEM_INSTANCE_NUMBER>`,
and indicate in `main.py` which `problem_instance` should be used.