#include "FlexiBarColumn.h"

FlexiBarColumn::FlexiBarColumn(const char * column_name)
{
  strcpy(this->column_name, column_name);
}

char * FlexiBarColumn::getName()
{
  return column_name;
}

FlexiBarColumn::~FlexiBarColumn()
{
  free( column_name );
}

