#ifndef __LINE_H
#define __LINE_H

class Line
{
   public:
      void setLength( double len );
      double getLength( void );
      Line();  // 这是构造函数
      ~Line();
 
   private:
      double length;
};

#endif
