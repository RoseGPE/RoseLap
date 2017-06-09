import ruamel.yaml

def recur_conv_to_float(o):
	if isinstance(o, int):
		return float(o)
	if isinstance(o, list):
		for i in range(len(o)):
			o[i] = recur_conv_to_float(o[i])
	if isinstance(o, dict):
		for key in o:
			o[key] = recur_conv_to_float(o[key])
	return o


def load(stream):
  out = ruamel.yaml.safe_load(stream)
  return recur_conv_to_float(out)