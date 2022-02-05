#ifndef MATRIX_READER
#define MATRIX_READER

//#include <string>
#include <iostream>
#include <fstream>
#include <string>
#include "stdlib.h"

using namespace std;

class MatrixReaderWriter{

public:	
	
	double* data;
	int rowNum;
	int columnNum;

	MatrixReaderWriter() = default;
	MatrixReaderWriter(double* data, int rownum, int columnNum);
	MatrixReaderWriter(const char* fileName);

	~MatrixReaderWriter();

	int load(const char* fileName);
	int save(const char* fileName);
};

#endif
