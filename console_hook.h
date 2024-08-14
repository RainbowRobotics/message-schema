#ifndef CONSOLE_HOOK_H
#define CONSOLE_HOOK_H

#include <iostream>
#include <streambuf>
#include <sstream>
#include <cstdarg>
#include <atomic>

#include <QObject>
#include <QTextEdit>
#include <QCheckBox>
#include <QScrollBar>
#include <QTextCursor>

class CONSOLE_HOOK : public std::streambuf
{
public:
    CONSOLE_HOOK(QTextEdit *_text_edit, QCheckBox *_ckb_auto)
    {
        this->text_edit = _text_edit;
        this->ckb_auto = _ckb_auto;
    }

protected:
    virtual int overflow(int c = EOF) override
    {
        if (c != EOF) {
            char buf = static_cast<char>(c);

            QScrollBar *v_scrollbar = text_edit->verticalScrollBar();
            int pre_val = v_scrollbar->value();

            text_edit->moveCursor(QTextCursor::End);
            text_edit->insertPlainText(QString(buf));

            if(ckb_auto->isChecked() == false)
            {
                v_scrollbar->setValue(pre_val);
            }
        }
        return c;
    }

private:
    QTextEdit *text_edit;
    QCheckBox *ckb_auto;
};

#define printf(...) hook_printf(__VA_ARGS__)
extern "C" void hook_printf(const char *format, ...);

#endif // CONSOLE_HOOK_H
