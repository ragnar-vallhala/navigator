#pragma once
// Small helpers that keep a side panel from being dragged narrower than its
// contents (which makes Qt crop widgets instead of refusing).

#include <QAction>
#include <QClipboard>
#include <QFontMetrics>
#include <QGuiApplication>
#include <QLabel>
#include <QPainter>
#include <QScrollArea>
#include <QScrollBar>
#include <QStyle>

namespace ui {

// Horizontal minimum width a scroll area needs so its (resizable) content is
// never squeezed below its own minimum: content hint + vertical scrollbar +
// frame. A plain QScrollArea reports a tiny minimum regardless of content.
inline int scrollAreaMinWidth(int contentMinWidth, int scrollBarWidth,
                              int frameWidth) {
  return contentMinWidth + scrollBarWidth + 2 * frameWidth;
}

// QScrollArea whose minimumSizeHint follows its content widget's, so a
// QSplitter / QStackedWidget parent cannot shrink it below the content.
class MinWidthScrollArea : public QScrollArea {
public:
  using QScrollArea::QScrollArea;

  QSize minimumSizeHint() const override {
    QSize s = QScrollArea::minimumSizeHint();
    if (const QWidget *w = widget()) {
      const int sb = style()->pixelMetric(QStyle::PM_ScrollBarExtent, nullptr,
                                          verticalScrollBar());
      s.setWidth(qMax(s.width(), scrollAreaMinWidth(w->minimumSizeHint().width(),
                                                    sb, frameWidth())));
    }
    return s;
  }
};

// Single-line label that elides ('…') to the width it is given instead of
// demanding room for the full text; the full text is the tooltip.
class ElidedLabel : public QLabel {
public:
  using QLabel::QLabel;

  void setFullText(const QString &t) {
    m_full = t;
    setToolTip(t);
    if (!m_copyAction) { // elided text can't be selected, so offer a copy menu
      m_copyAction = new QAction(QObject::tr("Copy full text"), this);
      QObject::connect(m_copyAction, &QAction::triggered, this, [this] {
        QGuiApplication::clipboard()->setText(m_full);
      });
      addAction(m_copyAction);
      setContextMenuPolicy(Qt::ActionsContextMenu);
    }
    updateGeometry();
    update();
  }
  QString fullText() const { return m_full; }

  // Preferred size is the whole string; the minimum is just an ellipsis, so the
  // label can shrink but still gets full width when there is room.
  QSize sizeHint() const override {
    return QSize(fontMetrics().horizontalAdvance(m_full) + 2,
                 QLabel::sizeHint().height());
  }
  QSize minimumSizeHint() const override {
    return QSize(fontMetrics().horizontalAdvance(QStringLiteral("…")) + 2,
                 QLabel::minimumSizeHint().height());
  }

protected:
  void paintEvent(QPaintEvent *) override {
    QPainter p(this);
    p.setPen(palette().color(foregroundRole()));
    p.drawText(contentsRect(), Qt::AlignLeft | Qt::AlignVCenter,
               fontMetrics().elidedText(m_full, Qt::ElideMiddle,
                                        contentsRect().width()));
  }

private:
  QString m_full;
  QAction *m_copyAction = nullptr;
};

} // namespace ui
