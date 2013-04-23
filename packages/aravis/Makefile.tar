TAR_DIR = build/aravis
TAR_URL = http://129.162.112.231/trac/RANGER/raw-attachment/wiki/ExternalLibraries/aravis.tar.gz

all: installed

installed:
	rm -rf build
	@mkdir -p build
	@mkdir -p lib
	@mkdir -p include
	cd build && wget $(TAR_URL)
	cd build && tar zxf aravis.tar.gz
	touch rospack_nosubdirs
	cd $(TAR_DIR) && ./autogen.sh
	cd $(TAR_DIR) && make
	cp $(TAR_DIR)/src/.libs/*.so* ./lib
	cp $(TAR_DIR)/src/*.h ./include
	touch installed

clean:
	rm -rf build installed lib include
