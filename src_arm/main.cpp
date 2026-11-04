#include <QApplication>
#include <QCommandLineParser>
#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QGuiApplication>
#include <QScreen>
#include <QTimer>
#include <QWindow>
#include <QString>
#include "mainwindow.h"
#include "detection_worker.h"
#include "eglfs_video_window.h"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>

#ifdef Q_OS_UNIX
#include <signal.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace {

volatile std::sig_atomic_t gQuitRequested = 0;
QtMessageHandler gPreviousQtMessageHandler = nullptr;

void displayAwareQtMessageHandler(QtMsgType type,
                                  const QMessageLogContext &context,
                                  const QString &message)
{
    if (gPreviousQtMessageHandler)
        gPreviousQtMessageHandler(type, context, message);
    else {
        const QByteArray formatted = qFormatLogMessage(type, context, message)
                                         .toLocal8Bit();
        std::fwrite(formatted.constData(), 1,
                    static_cast<std::size_t>(formatted.size()), stderr);
        std::fputc('\n', stderr);
        std::fflush(stderr);
    }

    if (message.contains(QStringLiteral("Could not queue DRM page flip"))
        && (message.contains(QStringLiteral("No space left on device"))
            || message.contains(QStringLiteral("Device or resource busy")))) {
        EglfsVideoWindow::notifyDrmPageFlipFailure();
    }
}

void requestGracefulQuit(int)
{
    gQuitRequested = 1;
}

#ifdef Q_OS_UNIX
void fatalSignalHandler(int signalNumber, siginfo_t *, void *)
{
    // Only async-signal-safe operations are allowed here.  The launcher will
    // translate 128+signal into a readable reason and collect dmesg/core data.
    switch (signalNumber) {
    case SIGSEGV: {
        static const char message[] = "[fatal-signal] SIGSEGV (11)\n";
        ::write(STDERR_FILENO, message, sizeof(message) - 1);
        break;
    }
    case SIGABRT: {
        static const char message[] = "[fatal-signal] SIGABRT (6)\n";
        ::write(STDERR_FILENO, message, sizeof(message) - 1);
        break;
    }
    case SIGBUS: {
        static const char message[] = "[fatal-signal] SIGBUS (7)\n";
        ::write(STDERR_FILENO, message, sizeof(message) - 1);
        break;
    }
    case SIGILL: {
        static const char message[] = "[fatal-signal] SIGILL (4)\n";
        ::write(STDERR_FILENO, message, sizeof(message) - 1);
        break;
    }
    case SIGFPE: {
        static const char message[] = "[fatal-signal] SIGFPE (8)\n";
        ::write(STDERR_FILENO, message, sizeof(message) - 1);
        break;
    }
    default: {
        static const char message[] = "[fatal-signal] UNKNOWN\n";
        ::write(STDERR_FILENO, message, sizeof(message) - 1);
        break;
    }
    }

    struct sigaction resetAction {};
    ::sigemptyset(&resetAction.sa_mask);
    resetAction.sa_handler = SIG_DFL;
    ::sigaction(signalNumber, &resetAction, nullptr);
    if (::kill(::getpid(), signalNumber) == 0)
        return;
    ::_exit(128 + signalNumber);
}

void installFatalSignalHandlers()
{
    // Ctrl-C also terminates an external `tee`.  Do not let the broken log
    // pipe terminate this process before MainWindow has stopped FPGA DMA.
    struct sigaction pipeAction {};
    ::sigemptyset(&pipeAction.sa_mask);
    pipeAction.sa_handler = SIG_IGN;
    ::sigaction(SIGPIPE, &pipeAction, nullptr);

    struct sigaction action {};
    ::sigemptyset(&action.sa_mask);
    action.sa_sigaction = fatalSignalHandler;
    action.sa_flags = SA_SIGINFO | SA_RESETHAND;
    const int fatalSignals[] = {SIGSEGV, SIGABRT, SIGBUS, SIGILL, SIGFPE};
    for (const int signalNumber : fatalSignals)
        ::sigaction(signalNumber, &action, nullptr);

    std::set_terminate([]() noexcept {
        static const char message[] = "[fatal-signal] std::terminate\n";
        ::write(STDERR_FILENO, message, sizeof(message) - 1);
        std::abort();
    });
}
#else
void installFatalSignalHandlers()
{
    std::set_terminate([]() noexcept { std::abort(); });
}
#endif

void sanitizePlatformArgument(int argc, char *argv[])
{
    for (int i = 1; i < argc; ++i) {
        char *arg = argv[i];
        if (!arg) continue;

        const bool platformValue =
            (i > 1 && std::strcmp(argv[i - 1], "-platform") == 0);
        const bool inlinePlatform =
            (std::strncmp(arg, "-platform=", 10) == 0);
        const std::size_t len = std::strlen(arg);
        if ((platformValue || inlinePlatform) && len > 0 && arg[len - 1] == '~') {
            // A trailing '~' is a common shell/copy typo ("eglfs~").  Qt parses
            // the platform before application code can report a friendlier error.
            arg[len - 1] = '\0';
        }
    }
}

void prepareQtRuntimeEnvironment()
{
#ifdef Q_OS_UNIX
    const uid_t uid = ::getuid();
    const QByteArray current = qgetenv("XDG_RUNTIME_DIR");
    struct stat info {};
    const bool currentUsable = !current.isEmpty()
        && ::lstat(current.constData(), &info) == 0
        && S_ISDIR(info.st_mode)
        && info.st_uid == uid
        && (info.st_mode & 0777) == 0700;

    if (!currentUsable) {
        const QByteArray runtimeDir =
            QByteArray("/tmp/camera-runtime-") + QByteArray::number(uid);
        QDir().mkpath(QString::fromLocal8Bit(runtimeDir));
        ::chmod(runtimeDir.constData(), 0700);
        qputenv("XDG_RUNTIME_DIR", runtimeDir);
    }

    const QByteArray kmsConfig = qgetenv("QT_QPA_EGLFS_KMS_CONFIG");
    if (!kmsConfig.isEmpty()
        && !QFileInfo::exists(QString::fromLocal8Bit(kmsConfig))) {
        // Let EGLFS auto-detect connectors instead of repeatedly opening a
        // stale /userdata/kms.json path.
        qunsetenv("QT_QPA_EGLFS_KMS_CONFIG");
    }
    if (!qEnvironmentVariableIsSet("QT_QPA_EGLFS_HIDECURSOR"))
        qputenv("QT_QPA_EGLFS_HIDECURSOR", "1");
#endif
}

} // namespace

int main(int argc, char *argv[])
{
    sanitizePlatformArgument(argc, argv);
    prepareQtRuntimeEnvironment();
    installFatalSignalHandlers();
    QApplication app(argc, argv);
    gPreviousQtMessageHandler = qInstallMessageHandler(
        displayAwareQtMessageHandler);

    // EGLFS may temporarily remove/close its only QOpenGLWindow when the HDMI
    // connector or KMS pipeline resets.  Do not convert that display event
    // into a clean process exit; record it and keep the capture service alive.
    app.setQuitOnLastWindowClosed(false);
    qWarning().nospace()
        << "[app-lifecycle] start pid=" << QCoreApplication::applicationPid()
        << " platform=" << app.platformName()
        << " quit_on_last_window_closed=" << app.quitOnLastWindowClosed()
        << " sigpipe=ignored";
    QObject::connect(&app, &QGuiApplication::lastWindowClosed, &app, []() {
        qCritical() << "[app-lifecycle] lastWindowClosed; automatic quit disabled";
    });
    QObject::connect(&app, &QGuiApplication::screenAdded, &app,
                     [](QScreen *screen) {
        qWarning().nospace()
            << "[app-lifecycle] screenAdded name="
            << (screen ? screen->name() : QStringLiteral("<null>"))
            << " hz=" << (screen ? screen->refreshRate() : 0.0);
    });
    QObject::connect(&app, &QGuiApplication::screenRemoved, &app,
                     [](QScreen *screen) {
        qCritical().nospace()
            << "[app-lifecycle] screenRemoved name="
            << (screen ? screen->name() : QStringLiteral("<null>"));
    });
    QObject::connect(&app, &QGuiApplication::focusWindowChanged, &app,
                     [](QWindow *window) {
        qWarning().nospace()
            << "[app-lifecycle] focusWindowChanged title="
            << (window ? window->title() : QStringLiteral("<null>"))
            << " visible=" << (window && window->isVisible())
            << " exposed=" << (window && window->isExposed());
    });
    QObject::connect(&app, &QGuiApplication::applicationStateChanged, &app,
                     [](Qt::ApplicationState state) {
        qWarning() << "[app-lifecycle] applicationStateChanged state="
                   << static_cast<int>(state);
    });
    QObject::connect(&app, &QCoreApplication::aboutToQuit, &app, []() {
        qCritical() << "[app-lifecycle] aboutToQuit emitted";
    });

    std::signal(SIGINT, requestGracefulQuit);
    std::signal(SIGTERM, requestGracefulQuit);
    QTimer signalPollTimer;
    QObject::connect(&signalPollTimer, &QTimer::timeout, &app, [&app]() {
        if (gQuitRequested) app.quit();
    });
    signalPollTimer.start(100);

    // 命令行参数: -wifi-host <ip> -wifi-port <port>
    QCommandLineParser parser;
    parser.setApplicationDescription("高速PCIe摄像头采集");
    parser.addHelpOption();
    QCommandLineOption wifiHostOpt(QStringList() << "w" << "wifi-host",
                                    "WiFi发送器IP地址", "ip");
    QCommandLineOption wifiPortOpt(QStringList() << "p" << "wifi-port",
                                    "WiFi发送器TCP端口", "port", "8080");
    QCommandLineOption ipCameraUrlOpt(QStringList() << "ip-camera-url",
                                      "手机IP摄像头HTTP MJPEG地址", "url");
    parser.addOption(wifiHostOpt);
    parser.addOption(wifiPortOpt);
    parser.addOption(ipCameraUrlOpt);
    parser.process(app);

    // 注册跨线程信号所需的自定义类型
    qRegisterMetaType<DetectResult>("DetectResult");

    app.setApplicationName("高速PCIe摄像头采集");
    app.setApplicationVersion("2.0");

    MainWindow window;
    QObject::connect(&app, &QCoreApplication::aboutToQuit,
                     &window, &MainWindow::prepareForShutdown,
                     Qt::DirectConnection);
    QObject::connect(&window, &QObject::destroyed, &app, []() {
        qCritical() << "[app-lifecycle] MainWindow destroyed";
    });

    // 如果指定了 WiFi 发送器, 设置连接参数
    if (parser.isSet(wifiHostOpt)) {
        QString host = parser.value(wifiHostOpt);
        quint16 port = static_cast<quint16>(parser.value(wifiPortOpt).toUInt());
        window.setWifiHost(host, port);
    }
    if (parser.isSet(ipCameraUrlOpt)) {
        QString error;
        if (!window.setIpCameraUrl(parser.value(ipCameraUrlOpt), &error)) {
            qCritical("Invalid --ip-camera-url: %s", qPrintable(error));
            return 2;
        }
    }
    const bool useEglfsFullscreen = app.platformName().contains("eglfs", Qt::CaseInsensitive);
    if (!(useEglfsFullscreen && window.usesDedicatedEglfsDisplay())) {
        if (useEglfsFullscreen)
            window.showFullScreen();
        else
            window.show();
    }

    const int exitCode = app.exec();
    qCritical() << "[app-lifecycle] event loop returned exit_code=" << exitCode;
    return exitCode;
}
