import numpy as np
import pandas as pd


def LoadSheet(df1):
    """
    Converts input dataframe into a meaningful dictionary of values

    Parameters
    ----------
    df1 : `dataframe`
        input sheet dataframe

    Returns
    -------
    prop_dict : `dict` of inputs

    """

    properties = df1["Parameters"].to_list()
    val = df1["Value"].to_list()
    prop_dict = {x: y for x, y in list(zip(properties, val))}
    return prop_dict


def nearestLength(a, b):
    """
    Takes two numbers a and b and tries to break a into 'n' integer parts with
    length close to b
    Parameters
    ----------
    a : `float`
        time in seconds
    b : `float`
        time in seconds

    Returns
    -------
    num , length : `tuple` of `float`

    """
    if a <= b:
        return 1, a
    ceil_var = 1.0 * np.ceil(a / b)
    floor_var = 1.0 * np.floor(a / b)
    dict_var = {}
    dict_var[abs(b - (a / ceil_var))] = a / ceil_var
    dict_var[abs(b - (a / floor_var))] = a / floor_var
    val = min(abs(b - (a / ceil_var)), abs(b - (a / floor_var)))
    dict_var[val]
    return a / dict_var[val], dict_var[val]
