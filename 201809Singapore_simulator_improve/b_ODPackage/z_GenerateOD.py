'''
Created on 2018/07/03
This module is used to generate a series of csv files containing of OD data in a given folder.
The generated csv files are used to check validity of ReadOD.py

The format of OD demand in the csv file consists of the following columns:
time slot, vehicle type, origin ID, destination ID,vehicle volume

Here, a series of csv files are created with their name indicating the time slot which is also used as the value of the first column in the csv file.

done and checked on 2018/7/3
@author: gong

'''
import csv,os,random
folderpath=r"D:\03Work during PD\31ERP2_in_COI\meso_by_python\OD_data"
list_time_slot=["7.00-7.04","7.05-7.09","7.10-7.14","7.15-7.19","7.20-7.24","7.25-7.29"]
list_origin     =[12,13,24,21,20]
list_destination=[1,2,4,5,18]
header=["time_slot","vehicle_type","Origin","Destination","volume"]
#volume between each pair of OD is randomly obtained from an interval of [10,100]
#we define three types of vehicles in this module
list_vehicle=["car","bus"]#"truck"]

for time_slot in list_time_slot:
    eachfile=time_slot+".csv"
    file_path=os.path.join('%s%s%s' % (folderpath,"/", eachfile))
    outfile=open(file_path,"ab")
    writer=csv.writer(outfile)
    writer.writerow(header)
    for vehicle in list_vehicle:
        for origin in list_origin:
            for destination in list_destination:
                if origin<>destination:
                    volume=0
                    if vehicle=="car":
                        volume=random.randint(3,8)
                    else:
                        volume=random.randint(1,2)
                    row=[time_slot,vehicle,origin,destination,volume]
                    writer.writerow(row)
                    row=[time_slot,vehicle,destination,origin,volume]# for the other direction of the link
                    writer.writerow(row)
                
    