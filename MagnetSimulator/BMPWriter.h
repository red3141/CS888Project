#include <string>
#include <iostream>
#include <fstream>

class BMPWriter{
private:
	std::ofstream imageFile;

public:
	BMPWriter(char* file);
	~BMPWriter();

	void writePixel(unsigned char red,
					unsigned char green,
					unsigned char blue);
};