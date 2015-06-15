#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char *argv[])
{
	char file[100];
	char target[100];
	char line[1000];
	FILE *fp1, *fp2;

	strcpy(file, argv[1]);
	strcpy(target, argv[2]);

	fp1 = fopen(file, "r");
	fp2 = fopen(target, "w");
	if (!fp1 || !fp2)
	{
		printf("Fail to open the file!\n");
		exit(-1);
	}
	
	while (fgets(line, 1000, fp1))
	{
		if (line[0] != '#')
			fputs(line, fp2);
	}

	fclose(fp1);
	fclose(fp2);

	return 0;
}
