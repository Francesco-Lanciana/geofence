from pykml.parser import Schema
from subprocess import STDOUT, check_call

def schema_invalid(file):
	'Check if it is a valid schema'
		with open(file, 'r') as f:
			#The files supplied are most likely part of the google extension schema, so validate it against that
			root = parser.fromstring(f.read())
			#Put the read cursor back at the start of the file
			f.seek(0)

		schema_gx = Schema("kml22gx.xsd")

		#If the kml file does not use a valid schema
		if not schema_gx.validate(root):
			return True
		else:
			return False



def validate_schema(file):
	'If file is invalid remove the cause'
	with open(file, 'r') as f:
		lines = f.readlines()
	with open(file, 'w') as f:
		for line in lines:
			if 'gx:drawOrder' not in line:
				f.write(line)
	return None

#For pykml make sure you have done: sudo apt-get install python-dev libxml2-dev libxslt1-dev zlib1g-dev
def install(module):
	'Allows the pip installer to install pykml if not already installed'
	#allows you to install necessary packages but needs password
	#proc = subprocess.Popen('sudo apt-get install python-dev libxml2-dev libxslt1-dev zlib1g-dev', shell=True, stdin=None, stdout=open(os.devnull,"wb"), stderr=STDOUT, executable="/bin/bash")
	#proc.wait()
	try:
		# because we want to import using a variable, do it this way
		module_obj = __import__(module)
	except ImportError, e:
		pip.main(['install', module])
		module_obj = __import__(module)

	# create a global object containging our module
	glob_pack = globals()[module] = module_obj


def globalise_module(module_name, module_obj):
	# create a global object containging our module
	glob_pack = globals()[module_name] = module_obj

