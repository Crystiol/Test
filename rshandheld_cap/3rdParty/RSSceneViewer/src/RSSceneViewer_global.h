#ifndef RSSCENEVIEWER_GLOBAL_H
#define RSSCENEVIEWER_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(RSSCENEVIEWER_LIBRARY)
#define RSSCENEVIEWER_EXPORT Q_DECL_EXPORT
#else
#define RSSCENEVIEWER_EXPORT Q_DECL_IMPORT
#endif

#endif // RSSCENEVIEWER_GLOBAL_H
