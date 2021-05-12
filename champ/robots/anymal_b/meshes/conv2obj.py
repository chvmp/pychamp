#coding:utf-8
class Model(object):
	def __init__(self,residues=[],args=None,style='strand'):
		self.v =[]	#顶点
		self.vn = []
		self.vt = []
		self.f = []	
	def exportObj(self, obj3d_path =''):
		obj_str = []		
		obj_str = obj_str + map(lambda v:'v %.3f %.3f %.3f\n' %tuple(v), self.v)
		obj_str = obj_str + map(lambda v:'vn %.3f %.3f %.3f\n' %tuple(v), self.vn)
		obj_str = obj_str + map(lambda v:'vt %.3f %.3f %.3f\n' %tuple(v), self.vt)
		if self.f:
			if len(self.f[0])==3:
				#f_str =  map(lambda f:'f ' + ' '.join(map(str,f)) +'\n'+'usemtl %s\n' %self.usemtl,self.f)
				f_str =  map(lambda f:'f ' + ' '.join(map(str,f)) +'\n',self.f)
			elif len(self.f[0])==6:				
				f_str = map(lambda f:'f %d//%d %d//%d %d//%d\n' %tuple(f), self.f)
				
				#obj_str = obj_str + map(lambda f:'f ' + ' '.join(map(str,f)) +'\n',self.f)
			obj_str = obj_str + f_str
		self.obj_str = obj_str
		if obj3d_path:
			open(obj3d_path,'w').writelines(obj_str)
		else:
			return obj_str
			
	@staticmethod
	def reduce(ma,mb):
		# ma,mb -> mc
		# usage: mc = reduce(Model.reduce, model_list)
		mc = Model()
		mc.v = (ma.v+mb.v)
		mc.vn = (ma.vn+mb.vn)
		mc.vt = (ma.vt+mb.vt)
		num_va = len(ma.v)
		num_vna = len(ma.vn)
		num_vta = len(ma.vt)
		f = mb.f
		if f:
			if len(f[0]) == 3:
				# 'f 1 2 3'
				f = map(lambda fi: [ x + num_va for x in fi], f)
			elif len(f[0]) == 6:
				# 'f 1//1 2//2 3//3'
				for fi in f:
					#f[::2] = f[::2] + num_va
					fi[::2] = [x + num_va for x in fi[::2]]
					#f[1::2] = f[1::2] + num_vn
					fi[1::2] = [x + num_vna for x in fi[1::2]]
			elif len(f[0]) == 9:
				# 'f 1/1/1 2/2/2 3/3/3'
				for fi in f:
					#f[::3] = f[::3] + num_va
					fi[::3] = [x + num_va for x in fi[::3]]
					#f[1::3] = f[1::3] + num_vt
					fi[1::3] = [x + num_vta for x in fi[1::3]]
					#f[2::3] = f[2::3] + num_vn
					fi[2::3] = [x + num_vna for x in fi[2::3]]
			mc.f = (ma.f + f)			
			#mc.set_style(ma.style+mb.style)
		return mc
	


try:
    import xml.etree.cElementTree as ET
except ImportError:
    import xml.etree.ElementTree as ET

inputfilename = './test.dae'
inputfilename = '5a63-ribbon.dae'
import sys
inputfilename = sys.argv[1]
outputfilename = sys.argv[2]

xmlns = "{http://www.collada.org/2005/11/COLLADASchema}"

#PARSE XML
# inputfilename: .dae file
# http://codingpy.com/article/parsing-xml-using-python/
tree = ET.ElementTree(file=inputfilename)
# methods:
#	tree.find('asset2/up_axis')


#FIX xmlns problem
# http://stackoverflow.com/questions/13412496/python-elementtree-module-how-to-ignore-the-namespace-of-xml-files-to-locate-ma
for el in tree.iter():
	if '}' in el.tag:
		el.tag = el.tag.split('}', 1)[1]  # strip all namespaces


#Parse COLLADA
# http://blog.csdn.net/zhouhangjay/article/details/8469085
library_geometries = tree.find('library_geometries')
#from ribbon2objv8 import *

# geometry_0
meshes = tree.findall('library_geometries/geometry/mesh')

#from ribbon2objv8 import *
models = []

for mesh in meshes:
	sources = mesh.findall('source')
	vertices = mesh.find('vertices')
	triangles = mesh.find('triangles')
	triangles_p = triangles.find('p')
	triangles_offset_dict = {}
	source_id_dict = {}
	for triangles_input in triangles.findall('input'):
		triangles_offset_dict[triangles_input.attrib['semantic']] = int(triangles_input.attrib['offset'])
		if triangles_input.attrib['semantic'] == 'VERTEX':
			source_id_dict['VERTEX'] = vertices.find('input').attrib['source'][1:]
		elif triangles_input.attrib['semantic'] == 'NORMAL':
			source_id_dict['NORMAL'] = triangles_input.attrib['source'][1:]	# 去除开头的'#'符号
	
	source_float_dict = {}
	for source in sources:
		float_array = map(float, source.find('float_array').text.split())
		source_float_dict[source.attrib['id']] = map(lambda i:float_array[i:i+3], range(0,len(float_array),3))

	model = Model()

	#model.set_v(source_float_dict[source_id_dict['VERTEX']])
	model.v = source_float_dict[source_id_dict['VERTEX']]

	#model.set_vn(source_float_dict[source_id_dict['NORMAL']])
	model.vn = source_float_dict[source_id_dict['NORMAL']]

	# f_list:[[v vn v vn v vn],[...]...]
	p_text_str = triangles_p.text.split()
	p_text = map(lambda idx:int(idx)+1, p_text_str)
	obj_f = map(lambda i:p_text[i:i+len(triangles_offset_dict)*3],range(0,len(p_text),len(triangles_offset_dict)*3))
	f_list = map(lambda f:[f[triangles_offset_dict['VERTEX']],f[triangles_offset_dict['NORMAL']],f[triangles_offset_dict['VERTEX']+len(triangles_offset_dict)],f[triangles_offset_dict['NORMAL']+len(triangles_offset_dict)],f[triangles_offset_dict['VERTEX']+len(triangles_offset_dict)*2],f[triangles_offset_dict['NORMAL']+len(triangles_offset_dict)*2]], obj_f)
	model.set_f(f_list)

	models.append(model)

model = reduce(Model.reduce, models)
#model.exportObj('dae2.obj')