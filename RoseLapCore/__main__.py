# import sys,os
# sys.path.append(os.path.dirname(__file__))

import input_processing
import batcher
import packer
# import argparse

# parser = argparse.ArgumentParser()
# parser.add_argument("file", help="name of batch config file")
# args = parser.parse_args()

print('configuring...')
# tests, vehicle, tracks, model, out = input_processing.process_input(args.file)
tests, vehicle, tracks, model, out = input_processing.process_input("test_batch.yaml")
print('batching...')
results = batcher.batch(tests, vehicle, tracks, model, out[1] != 0)
print('packing...')
print(results)
packer.pack(results, out[0])
print('done!')