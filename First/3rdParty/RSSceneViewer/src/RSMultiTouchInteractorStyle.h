#ifndef RSMULTITOUCHINTERACTORSTYLE_H
#define RSMULTITOUCHINTERACTORSTYLE_H

#include <vtkInteractorStyleMultiTouchCamera.h>

class RSMultiTouchInteractorStyle : public vtkInteractorStyleMultiTouchCamera
{
public:
    static RSMultiTouchInteractorStyle* New();
    vtkTypeMacro(RSMultiTouchInteractorStyle, vtkInteractorStyleMultiTouchCamera);

	void GetPickPosition(double pickPosition[4]);

	void OnRotate() override;
	void OnPinch() override;
	void OnPan() override;

	void OnStartSwipe();
	void OnSwipe();
	void OnEndSwipe();
	void OnLongTap();
	void OnTap();

protected:
    RSMultiTouchInteractorStyle();
    ~RSMultiTouchInteractorStyle(){}

private:
	bool IsSwiping = false;

private:
    RSMultiTouchInteractorStyle(const RSMultiTouchInteractorStyle&) = delete;
    void operator=(const RSMultiTouchInteractorStyle&) = delete;
};

#endif // RSMULTITOUCHINTERACTORSTYLE_H
