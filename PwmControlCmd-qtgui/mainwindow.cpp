#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    pwmVal = 0;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_lineEdit_input_textEdited(const QString &arg1)
{

    pwmVal =    arg1.toInt();
    if(pwmVal<0) pwmVal = 0;
    else if(pwmVal>1024) pwmVal = 1024;

    this->ui->horizontalSlider->setValue(pwmVal);
    /**change slider and lineEdit_show**/
    this->updateCmdContent();
}

void MainWindow::updateCmdContent()
{
     static quint8 pwmCmd[6] = {0xa5, 0xf0, 0x02, 0x00, 0x00, 0x57};
     QString showVal = "0x";

     pwmCmd[3] = quint8(pwmVal&0xff);
     pwmCmd[4] = quint8((pwmVal>>8)&0xff);
     pwmCmd[5] = 0x00;
     for(quint8 i = 0; i<5;i++){
         pwmCmd[5] ^= pwmCmd[i];
         //QString dataHex = QString::number(pwmCmd[i], 16);
         QString dataHex = QString("%1").arg(pwmCmd[i],2,16,QLatin1Char('0'));
         showVal += (dataHex + " 0x");
     }
     //showVal += QString::number(pwmCmd[5], 16);
     showVal += QString("%1").arg(pwmCmd[5],2,16,QLatin1Char('0'));
     /*show on LineEdit_show*/
     this->ui->lineEdit_show->setText(showVal);
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    pwmVal =    value;
    QString str = QString::number(pwmVal, 10);
    this->ui->lineEdit_input->setText(str);
    /**change slider and lineEdit_show**/
    this->updateCmdContent();
}

void MainWindow::on_pushButton_clicked()
{

    QClipboard *clipboard = QApplication::clipboard();
    //QString originalText = clipboard->text();
    QString newText = this->ui->lineEdit_show->text();
    clipboard->setText(newText);
}
