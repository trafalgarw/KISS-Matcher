cppinstall:
	@mkdir -p cpp/kiss_matcher/build
	@cmake -Bcpp/kiss_matcher/build cpp/kiss_matcher -DCMAKE_BUILD_TYPE=Release
	@cmake --build cpp/kiss_matcher/build -j$(nproc --all)
	@sudo cmake --install cpp/kiss_matcher/build
