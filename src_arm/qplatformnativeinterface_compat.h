#ifndef ARM_FINAL_QPLATFORMNATIVEINTERFACE_COMPAT_H
#define ARM_FINAL_QPLATFORMNATIVEINTERFACE_COMPAT_H

// ABI-compatible Qt 5 QPA declaration used only to call the EGLFS KMS
// integration's documented-in-source "dri_fd" native resource.  Keeping the
// declaration local avoids requiring the optional qtbase5-private-dev package
// on the RK3568 build image.
#include <QtGui/qtguiglobal.h>
#include <QtGui/qwindowdefs.h>
#include <QtCore/QObject>
#include <QtCore/QVariant>

QT_BEGIN_NAMESPACE

class QOpenGLContext;
class QScreen;
class QWindow;
class QPlatformWindow;
class QBackingStore;
class QCursor;

class Q_GUI_EXPORT QPlatformNativeInterface : public QObject
{
public:
    virtual void *nativeResourceForIntegration(const QByteArray &resource);
    virtual void *nativeResourceForContext(const QByteArray &resource,
                                           QOpenGLContext *context);
    virtual void *nativeResourceForScreen(const QByteArray &resource,
                                          QScreen *screen);
    virtual void *nativeResourceForWindow(const QByteArray &resource,
                                          QWindow *window);
    virtual void *nativeResourceForBackingStore(const QByteArray &resource,
                                                QBackingStore *backingStore);
#ifndef QT_NO_CURSOR
    virtual void *nativeResourceForCursor(const QByteArray &resource,
                                          const QCursor &cursor);
#endif

    typedef void *(*NativeResourceForIntegrationFunction)();
    typedef void *(*NativeResourceForContextFunction)(QOpenGLContext *context);
    typedef void *(*NativeResourceForScreenFunction)(QScreen *screen);
    typedef void *(*NativeResourceForWindowFunction)(QWindow *window);
    typedef void *(*NativeResourceForBackingStoreFunction)(QBackingStore *backingStore);
    virtual NativeResourceForIntegrationFunction
        nativeResourceFunctionForIntegration(const QByteArray &resource);
    virtual NativeResourceForContextFunction
        nativeResourceFunctionForContext(const QByteArray &resource);
    virtual NativeResourceForScreenFunction
        nativeResourceFunctionForScreen(const QByteArray &resource);
    virtual NativeResourceForWindowFunction
        nativeResourceFunctionForWindow(const QByteArray &resource);
    virtual NativeResourceForBackingStoreFunction
        nativeResourceFunctionForBackingStore(const QByteArray &resource);
    virtual QFunctionPointer platformFunction(const QByteArray &function) const;

    virtual QVariantMap windowProperties(QPlatformWindow *window) const;
    virtual QVariant windowProperty(QPlatformWindow *window,
                                    const QString &name) const;
    virtual QVariant windowProperty(QPlatformWindow *window,
                                    const QString &name,
                                    const QVariant &defaultValue) const;
    virtual void setWindowProperty(QPlatformWindow *window,
                                   const QString &name,
                                   const QVariant &value);
};

QT_END_NAMESPACE

#endif
