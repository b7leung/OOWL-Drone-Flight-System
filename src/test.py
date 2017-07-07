class test(object):
	def __init__(self):
		self.a=None
		print(self.a)
		self.change()
		print(self.a)
	def change(self):
		self.a=1
	
if __name__=='__main__':
	b=test()
	
