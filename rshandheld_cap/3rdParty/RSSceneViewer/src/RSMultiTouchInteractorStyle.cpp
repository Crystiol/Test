#include "RSMultiTouchInteractorStyle.h"
#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <vtkObjectFactory.h>

vtkStandardNewMacro(RSMultiTouchInteractorStyle);

void RSMultiTouchInteractorStyle::GetPickPosition(double pickPosition[4])
{
	vtkCamera* camera = this->CurrentRenderer->GetActiveCamera();
	if (!camera)
	{
		return;
	}

	int pointer = this->Interactor->GetPointerIndex();

	this->FindPokedRenderer(this->Interactor->GetEventPositions(pointer)[0],
		this->Interactor->GetEventPositions(pointer)[1]);

	double* focalPointWorld = camera->GetFocalPoint();
	double focalPointDisplay[3] = { 0, 0, 0 };
	vtkInteractorObserver::ComputeWorldToDisplay(this->CurrentRenderer, focalPointWorld[0],
		focalPointWorld[1], focalPointWorld[2], focalPointDisplay);

	int* touchPositionDisplay = this->Interactor->GetEventPositions(pointer);
	double pickPoint[4] = { 0, 0, 0, 0 };
	vtkInteractorObserver::ComputeDisplayToWorld(this->CurrentRenderer, touchPositionDisplay[0],
		touchPositionDisplay[1], focalPointDisplay[2], pickPosition);
}

void RSMultiTouchInteractorStyle::OnRotate()
{
	if (this->IsSwiping)
	{
		return;
	}
	Superclass::OnRotate();
}

void RSMultiTouchInteractorStyle::OnPinch()
{
	if (this->IsSwiping)
	{
		return;
	}
	Superclass::OnPinch();
}

void RSMultiTouchInteractorStyle::OnPan()
{
	if (this->IsSwiping)
	{
		return;
	}
	Superclass::OnPan();
}
	
void RSMultiTouchInteractorStyle::OnStartSwipe()
{
	this->IsSwiping = true;
	this->StartGesture();
}

void RSMultiTouchInteractorStyle::OnSwipe()
{
	if (!this->CurrentRenderer)
	{
		return;
	}

	double hsv[3] = { this->Interactor->GetRotation() / 360.0, 1.0, 1.0 };
	double rgb[3];
	vtkMath::HSVToRGB(hsv, rgb);

	this->CurrentRenderer->Render();
}

void RSMultiTouchInteractorStyle::OnEndSwipe()
{
	this->IsSwiping = false;
	this->EndGesture();
}

void RSMultiTouchInteractorStyle::OnLongTap()
{
	if (!this->CurrentRenderer)
	{
		return;
	}

	vtkCamera* camera = this->CurrentRenderer->GetActiveCamera();
	if (!camera)
	{
		return;
	}

	camera->SetParallelProjection(!camera->GetParallelProjection());

	double pickPoint[4] = { 0, 0, 0, 0 };
	this->GetPickPosition(pickPoint);

	this->CurrentRenderer->Render();
}

void RSMultiTouchInteractorStyle::OnTap()
{
	if (!this->CurrentRenderer)
	{
		return;
	}

	this->CurrentRenderer->SetBackground(
		(double)rand() / RAND_MAX, (double)rand() / RAND_MAX, (double)rand() / RAND_MAX);

	double pickPoint[4] = { 0, 0, 0, 0 };
	this->GetPickPosition(pickPoint);

	this->CurrentRenderer->Render();
}

RSMultiTouchInteractorStyle::RSMultiTouchInteractorStyle()
{

}
