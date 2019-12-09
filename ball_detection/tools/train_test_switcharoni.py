import os
import sys
import random

#path= sys.argv[1]
path= "/Users/eric/Desktop/ball/data/ball_labels.csv"
path= os.path.abspath( path)

train= path[:path.index( path.split( "/")[-1])] + "train_" + path.split( "/")[-1]
test= path[:path.index( path.split( "/")[-1])] + "test_" + path.split( "/")[-1]

with open( path, "r") as data, open( train, "w") as train, open( test, "w") as test:
    train_data= list()
    test_data= list()
    header= data.readline()
    for line in data.readlines():
        train_data.append( line)
    for n in range( int( len( train_data) / 10)):
        test_data.append( 
            train_data.pop( 
                random.randint( 0, len( train_data)-1)
            )
        )
    train_data.insert( 0, header)
    test_data.insert( 0, header)
    train.writelines( train_data)
    test.writelines( test_data)