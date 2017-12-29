import marshal
import platform

def pack(results, filename):
	b = marshal.dumps(results)
	filename = toRslpFilename(filename)

	with open(filename, 'wb') as f:
		f.write(b)

def unpack(filename):
	with open(filename, 'rb') as f:
		return marshal.load(f)
	
def toRslpFilename(filename):
	return 'out/' + filename + '-' + platform.python_version() + '.rslp'