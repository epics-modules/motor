#!/usr/bin/env python

# After editing the .opi file so menu items that should replace a display
# actually do replace, this program can be used to change all "OPEN DISPLAY"
# actions that do not replace with "OPEN_OPI_IN_VIEW" actions
import sys
def main():
	if len(sys.argv) < 2:
		print "usage: convertMenu.py filename"
		return
	filename = sys.argv[1]
	file = open(filename,"r+")
	lines=file.readlines()
	for i in range(len(lines)):
		ix = lines[i].find("OPEN_DISPLAY")
		if ix > 0:
			print "found OPEN_DISPLAY:", lines[i]
			found = False
			for j in range(i, i+10):
				ix = lines[j].find("<replace>false</replace>")
				if ix > 0:
					print "found <replace>false</replace>:", lines[j]
					found = True
					break
			if found:
				lines[i] = lines[i].replace("OPEN_DISPLAY", "OPEN_OPI_IN_VIEW")
				lines[j] = lines[j].replace("<replace>false</replace>", "<Position>1</Position>")
				i = j+1
	file.seek(0)
	file.writelines(lines)

if __name__ == "__main__":
	main()
