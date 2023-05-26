# importing the pandas library
import pandas as pd

# reading the csv file
df = pd.read_csv("1ms/wind_at_cell_centers_0.csv")

# updating the column value/data
df["U:0"]=0
df["U:1"]=0
df["U:2"]=0


# # writing into the file
df.to_csv("w0/wind_at_cell_centers_0.csv", index=False)

# print(df)
