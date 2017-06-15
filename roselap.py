"""
Usage:
  roselap.py run <filename> [--no_output]
  roselap.py load <filename> [--no_output]
  roselap.py meshconv <filename>
  roselap.py -h | --help
  roselap.py --version

Options:
  -h --help     Show this screen.
  --version     Show version.
  --no_output   Don't display any plots.
"""

import study
import meshconv
import datetime

from docopt import docopt

def run(filename, no_plot):
    s = study.run(filename)
    s.save()
    if not no_plot:
        s.plot()
    return s

def load(filename, no_plot):
    s = study.load(filename)
    print('Study was conducted between: '
        +datetime.datetime.fromtimestamp(s.timestamp_start).strftime('%Y-%m-%d %H:%M:%S')
        +' and '+datetime.datetime.fromtimestamp(s.timestamp_end).strftime('%Y-%m-%d %H:%M:%S')
        + ' (took ' + str(datetime.timedelta(seconds=s.timestamp_end-s.timestamp_start)) + ')')
    print('Please be sure that your study file and referenced vehicle file(s) are older than this.')

    if not no_plot:
        s.plot()
    return s


def run_meshconv(filename):
    meshconv.run(filename)

if __name__ == '__main__':
    arguments = docopt(__doc__, version='3.3')
    if arguments['run']:
        s=run(arguments['<filename>'], arguments['--no_output'])
    elif arguments['load']:
        s=load(arguments['<filename>'], arguments['--no_output'])
    elif arguments['meshconv']:
        run_meshconv(arguments['<filename>'])

'''
better detail plot titles
data toggle
optional plot points

'''