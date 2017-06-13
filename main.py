"""
Usage:
  roselap.py run <filename> [--no_output]
  roselap.py load <filename> [--no_output]
Options:
  -h --help     Show this screen.
  --version     Show version.
"""


from sim import *
import study

from docopt import docopt

def run(filename, no_plot):
    s = study.run(filename)
    s.save()
    if not no_plot:
        s.plot()
    return s

def load(filename):
    s = study.load(filename)
    if not no_plot:
        s.plot()
    return s

if __name__ == '__main__':
    arguments = docopt(__doc__, version='3.3')
    if arguments['run']:
        s=run(arguments['<filename>'], arguments['--no_output'])
    elif arguments['load']:
        s=load(arguments['<filename>'], arguments['--no_output'])

'''
better detail plot titles
data toggle
optional plot points

'''