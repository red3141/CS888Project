#include "BMPWriter.h"

using namespace std;

// This class writes 24-bit BMP files for 720x480 images.

BMPWriter::BMPWriter(char* file) {
	imageFile.open(file, ios::binary);

	imageFile << 'B';
	imageFile << 'M';
	// image size
	imageFile << (char)0x36;
	imageFile << (char)0xD2;
	imageFile << (char)0x0F;
	imageFile << (char)0;
	// reserved
	imageFile << (char)0;
	imageFile << (char)0;
	imageFile << (char)0;
	imageFile << (char)0;
	// starting address of image data
	imageFile << (char)0x36;
	imageFile << (char)0;
	imageFile << (char)0;
	imageFile << (char)0;
	// size of this header
	imageFile << (char)0x28;
	imageFile << (char)0;
	imageFile << (char)0;
	imageFile << (char)0;
	// width in pixels
	imageFile << (char)0xD0;
	imageFile << (char)0x02;
	imageFile << (char)0;
	imageFile << (char)0;
	// height in pixels
	imageFile << (char)0xE0;
	imageFile << (char)0x01;
	imageFile << (char)0;
	imageFile << (char)0;
	// number of colour planes
	imageFile << (char)0x01;
	imageFile << (char)0;
	// number of bits per pixel
	imageFile << (char)0x18;
	imageFile << (char)0;
	// no pixel array compression
	imageFile << (char)0;
	imageFile << (char)0;
	imageFile << (char)0;
	imageFile << (char)0;
	// size of the raw bitmap data
	imageFile << (char)0x00;
	imageFile << (char)0xD2;
	imageFile << (char)0x0F;
	imageFile << (char)0;
	// print resolution horizontal (2835 px/m)
	imageFile << (char)0x13;
	imageFile << (char)0x0B;
	imageFile << (char)0;
	imageFile << (char)0;
	// print resolution vertical (2835 px/m)
	imageFile << (char)0x13;
	imageFile << (char)0x0B;
	imageFile << (char)0;
	imageFile << (char)0;
	// number of colours in palatte
	imageFile << (char)0;
	imageFile << (char)0;
	imageFile << (char)0;
	imageFile << (char)0;
	// all colours are important
	imageFile << (char)0;
	imageFile << (char)0;
	imageFile << (char)0;
	imageFile << (char)0;
}

BMPWriter::~BMPWriter() {
	imageFile.close();
}

void BMPWriter::writePixel(unsigned char red,
						   unsigned char green,
						   unsigned char blue) {
	imageFile << blue;
	imageFile << green;
	imageFile << red;
}