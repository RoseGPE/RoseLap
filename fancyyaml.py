import ruamel.yaml
import numpy as np

def recur_conv_to_float(o):
	if isinstance(o, int):
		return float(o)
	if isinstance(o, list):
		for i in range(len(o)):
			o[i] = recur_conv_to_float(o[i])
	if isinstance(o, dict):
		rs = None
		re = None
		rd = None
		for key in o:
			if key == 'range_start':
				rs = o[key]
			elif key == 'range_end':
				re = o[key]
			elif key == 'range_step':
				rd = o[key]

			o[key] = recur_conv_to_float(o[key])
		if rs != None and re != None and rd != None:
			return list(np.arange(rs,re+rd, rd))
	return o


def load(stream):
  out = ruamel.yaml.safe_load(stream)
  return recur_conv_to_float(out)