# S.A. Velastin 2021
# Does some sanity checks on object coordinates

import os
import logging

def in_range (valuename, value, maximum, class_name, filename='', entry_values=[]):
    log = logging.getLogger()
    if filename != '':
        filename=os.path.basename(filename) # else we dont have a filename to report
    if value < 0:
        value = 0
        log.error (f'*** {valuename} <0: {class_name} {filename} {entry_values}')
    if value >= maximum:
        value = maximum-1
        #log.error (f'*** {valuename} >= {maximum} {class_name} {filename} {entry_values}')
        log.error (f'*** {valuename} >= {maximum} {class_name} {filename} {entry_values} >> Value adjusted successfully')
    return value
        
# Checks the validity of the bbox returning new values if necessary
def sanity_check (xmin, xmax, ymin, ymax, Image_width, Image_height, class_name, filename=''):
    log = logging.getLogger()
    entry_values =[xmin,xmax,ymin,ymax,Image_width, Image_height]
    if filename != '':
        filename=os.path.basename(filename) # else we dont have a filename to report
    xmin = in_range('xmin', xmin, Image_width, class_name, filename, entry_values)
    xmax = in_range('xmax', xmax, Image_width, class_name, filename, entry_values)
    ymin = in_range('ymin', ymin, Image_height, class_name, filename, entry_values)
    ymax = in_range('ymax', ymax, Image_height, class_name, filename, entry_values)
    if xmin>xmax:
        xmin,xmax = xmax,xmin   #swap
        log.error (f'*** xmax < xmin: {class_name} {filename} {entry_values}')
    if ymin>ymax:
        ymin,ymax = ymax,ymin   #swap
        log.error (f'*** ymax < ymin: {class_name} {filename} {entry_values}')
    return xmin,xmax,ymin,ymax
