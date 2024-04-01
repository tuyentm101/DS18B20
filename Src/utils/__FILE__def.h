/*
 * __FILE__def.h
 *
 * Author: Manh Tuyen Ta
 */

#ifndef FILE__DEF_H_
#define FILE__DEF_H_

/**
 * Redefine the__FILE__ macro so it contains just the file name
 * Add -Wno-builtin-macro-redefined to compiler options to suppress the warning about this.
 */


#define __FILE__ (__builtin_strrchr("/" __BASE_FILE__, '/') + 1)


#endif /* FILE__DEF_H_ */
