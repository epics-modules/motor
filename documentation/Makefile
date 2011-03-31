HTMLFLAGS = --book --duplex --no-numbered --format pdf14 --title --size letter

%.pdf: %.html
	-htmldoc $(HTMLFLAGS) --outfile $@ $<

all: doxygen install

doxygen:
	doxygen

install:
	cp -f *.html /home/epics/web_software/epics
	cp -f *.png /home/epics/web_software/epics
	cp -rfp motorDoxygenHTML  /home/epics/web_software/epics/

clean:
	rm -rf motorDoxygenHTML/*


realclean: clean
