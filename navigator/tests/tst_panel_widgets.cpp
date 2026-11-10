// Side-panel sizing helpers: a scroll area in a splitter must not be shrunk
// below its content's minimum width, and an ElidedLabel must not widen it.

#include "ui/widgets/PanelWidgets.h"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSplitter>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QtTest>

class PanelWidgetsTest : public QObject {
  Q_OBJECT
private slots:
  void minWidthFormula() {
    QCOMPARE(ui::scrollAreaMinWidth(300, 14, 1), 316);
  }

  void splitterCannotCropPanel() {
    QSplitter split(Qt::Horizontal);
    split.setChildrenCollapsible(false);
    split.addWidget(new QWidget);
    auto *stack = new QStackedWidget;
    split.addWidget(stack);

    // Page with real minimum-width content plus a very long path label.
    auto *page = new QWidget;
    auto *v = new QVBoxLayout(page);
    auto *row = new QHBoxLayout;
    auto *btn = new QPushButton(QStringLiteral("A reasonably wide button"));
    row->addWidget(btn);
    row->addWidget(new QLineEdit(QStringLiteral("some text")));
    v->addLayout(row);
    auto *elided = new ui::ElidedLabel;
    const QString longPath =
        QStringLiteral("/very/long/directory/name/").repeated(12);
    elided->setFullText(longPath);
    v->addWidget(elided);
    auto *scroll = new ui::MinWidthScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setWidget(page);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    stack->addWidget(scroll);

    split.resize(900, 500);
    split.show();
    QVERIFY(QTest::qWaitForWindowExposed(&split));

    const int need = scroll->minimumSizeHint().width();
    QVERIFY2(need >= page->minimumSizeHint().width(),
             "scroll area hint must cover content");
    // The long path must not dictate the width.
    QVERIFY2(page->minimumSizeHint().width() < 400,
             qPrintable(QString::number(page->minimumSizeHint().width())));

    // Drag the handle as far right as it goes.
    split.setSizes({10000, 1});
    QCoreApplication::processEvents();
    QVERIFY2(stack->width() >= need,
             qPrintable(QStringLiteral("panel %1 < need %2")
                            .arg(stack->width())
                            .arg(need)));
    // Nothing clipped horizontally: content got at least its minimum width.
    QVERIFY(page->width() >= page->minimumSizeHint().width());
    QVERIFY(btn->geometry().right() <= page->width());
    QCOMPARE(elided->toolTip(), longPath);
  }

  void wrappedRowIsNarrowerAndSwitchesLive() {
    QWidget page;
    auto *grid = new QGridLayout(&page);
    QList<QWidget *> row;
    for (const char *t : {"RC transmitter", "USB joystick", "/dev/ttyUSB0",
                          "115200", "Connect", "Acro (ch6)"})
      row << new QPushButton(QString::fromLatin1(t), &page);
    ui::layoutWrappedRow(grid, row, false, row[2]);
    const int normal = page.minimumSizeHint().width();
    ui::layoutWrappedRow(grid, row, true, row[2]);
    const int compact = page.minimumSizeHint().width();
    QVERIFY2(compact < normal, qPrintable(QStringLiteral("%1 !< %2")
                                              .arg(compact).arg(normal)));
    QCOMPARE(grid->count(), row.size());
    QCOMPARE(grid->rowCount(), 2);
    ui::layoutWrappedRow(grid, row, false, row[2]);
    QCOMPARE(page.minimumSizeHint().width(), normal);
  }
};

QTEST_MAIN(PanelWidgetsTest)
#include "tst_panel_widgets.moc"
