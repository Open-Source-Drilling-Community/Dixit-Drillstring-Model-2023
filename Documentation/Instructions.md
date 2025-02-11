# Dixit-Drillstring-Model
Based on the 2023 Dixit Drillstring Model.  Modifications have been made to 


## Required Packages
* Numpy
* Scipy
* Pandas
* Matplotlib
* Plotly

## Benchmarking
Changes:
* Added Coulomb friction model
* Optimized the model

## Output
Output files can be located in drillsim-main/Output folder

## Running
In order to run the simulations, simply run the first cell in `Run.ipynb` notebook file which is located in `drillsim-main` folder.  You need to pass friction model as an argument (`stribeck` or `coulomb`) in main_driver() function at the end of 1st cell (Line 15). There is also an additional option 'save_pickle' which you can set to TRUE or FALSE. If you don't need to work on data after simulation (create plots, etc.) then simply set it to FALSE; it will save some time, and will run approximately 10-20 seconds faster.

Input files are in `Tests` folder.
1.	In order to run the code, you need to get the input files from the test folder.
2.	Copy that input file and paste it to `drillsim-main/Input/`.
3.	Run the `Run.ipynb` file.
4.	You need to run first two cells.
5.	Afterwards you can go to `drillsim-main/Output/` folder and the main plots will be in `Outputs.html` file.

- The rest of the code in `Run.ipynb` file is for visualization purposes.
- If you want to export the data as CSV you need to run cells which are below `EXPORT CSV` in the `RUN.ipynb`.