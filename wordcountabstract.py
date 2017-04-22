
import subprocess
proc=subprocess.Popen('/Users/Pau/Downloads/TeXcount_3_0_0_24/texcount.pl -brief /Users/Pau/Desktop/Extended_Essay/Abstract.tex ', shell=True, stdout=subprocess.PIPE, )
output=proc.communicate()[0]
output=output.split('+')
output=output[0]
output=int(output)-2 ## So it doesn't count the words 'word count'
output=str(output)
f = open('/Users/Pau/Desktop/Extended_Essay/wordcountabstract.txt','w')
f.write(output)
f.close()

