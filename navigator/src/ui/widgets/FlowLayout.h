#pragma once
// A row layout that wraps onto further lines when it is narrower than its
// items need. Its minimum width is that of the widest single item, so a row of
// buttons / check boxes no longer sets the floor for the panel it sits in.

#include <QLayout>
#include <QList>
#include <QRect>
#include <QSize>
#include <QWidget>
#include <QtGlobal>

namespace ui {

class FlowLayout : public QLayout {
public:
  explicit FlowLayout(QWidget *parent = nullptr, int spacing = 6)
      : QLayout(parent), m_spacing(spacing) {
    setContentsMargins(0, 0, 0, 0);
  }
  ~FlowLayout() override {
    while (QLayoutItem *it = takeAt(0))
      delete it;
  }

  void addItem(QLayoutItem *item) override {
    if (QLayout *sub = item->layout())
      addChildLayout(sub); // reparents its widgets, as QBoxLayout::addLayout does
    m_items.append(item);
  }
  int count() const override { return int(m_items.size()); }
  QLayoutItem *itemAt(int i) const override {
    return (i >= 0 && i < m_items.size()) ? m_items.at(i) : nullptr;
  }
  QLayoutItem *takeAt(int i) override {
    return (i >= 0 && i < m_items.size()) ? m_items.takeAt(i) : nullptr;
  }
  Qt::Orientations expandingDirections() const override { return {}; }
  bool hasHeightForWidth() const override { return true; }
  int heightForWidth(int w) const override { return doLayout(QRect(0, 0, w, 0), true); }
  void setGeometry(const QRect &r) override {
    QLayout::setGeometry(r);
    doLayout(r, false);
  }
  QSize sizeHint() const override { return minimumSize(); }
  QSize minimumSize() const override {
    QSize s;
    for (const QLayoutItem *it : m_items)
      s = s.expandedTo(it->minimumSize());
    const QMargins m = contentsMargins();
    return s + QSize(m.left() + m.right(), m.top() + m.bottom());
  }

private:
  int doLayout(const QRect &rect, bool measureOnly) const {
    const QMargins m = contentsMargins();
    const QRect area = rect.adjusted(m.left(), m.top(), -m.right(), -m.bottom());
    int x = area.x();
    int y = area.y();
    int lineH = 0;
    for (QLayoutItem *it : m_items) {
      if (it->isEmpty())
        continue;
      const QSize hint = it->sizeHint().expandedTo(it->minimumSize());
      int w = qMin(hint.width(), qMax(area.width(), it->minimumSize().width()));
      if (x > area.x() && x + w > area.right() + 1) { // wrap
        x = area.x();
        y += lineH + m_spacing;
        lineH = 0;
      }
      if (!measureOnly)
        it->setGeometry(QRect(QPoint(x, y), QSize(w, hint.height())));
      x += w + m_spacing;
      lineH = qMax(lineH, hint.height());
    }
    return y + lineH - rect.y() + m.bottom();
  }

  QList<QLayoutItem *> m_items;
  int m_spacing;
};

} // namespace ui
