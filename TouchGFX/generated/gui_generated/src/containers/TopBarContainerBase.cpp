/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/containers/TopBarContainerBase.hpp>
#include <images/BitmapDatabase.hpp>
#include <texts/TextKeysAndLanguages.hpp>
#include <touchgfx/Color.hpp>

TopBarContainerBase::TopBarContainerBase() :
    chromArtPressedCallback(0),
    buttonCallback(this, &TopBarContainerBase::buttonCallbackHandler)
{
    setWidth(800);
    setHeight(64);
    background.setXY(0, 0);
    background.setBitmap(touchgfx::Bitmap(BITMAP_TOP_BAR_ID));
    add(background);

    mcuTitle.setXY(248, 10);
    mcuTitle.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    mcuTitle.setLinespacing(0);
    mcuTitle.setTypedText(touchgfx::TypedText(T___SINGLEUSE_PI18));
    add(mcuTitle);

    mcuValue.setPosition(291, 10, 60, 32);
    mcuValue.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    mcuValue.setLinespacing(0);
    Unicode::snprintf(mcuValueBuffer, MCUVALUE_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_4750).getText());
    mcuValue.setWildcard(mcuValueBuffer);
    mcuValue.setTypedText(touchgfx::TypedText(T___SINGLEUSE_Z2RU));
    add(mcuValue);

    fpsTite.setXY(448, 10);
    fpsTite.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    fpsTite.setLinespacing(0);
    fpsTite.setTypedText(touchgfx::TypedText(T___SINGLEUSE_ZT74));
    add(fpsTite);

    fpsValue.setPosition(482, 10, 43, 32);
    fpsValue.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    fpsValue.setLinespacing(0);
    Unicode::snprintf(fpsValueBuffer, FPSVALUE_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_7OUF).getText());
    fpsValue.setWildcard(fpsValueBuffer);
    fpsValue.setTypedText(touchgfx::TypedText(T___SINGLEUSE_LB3O));
    add(fpsValue);

    infoButton.setXY(385, 10);
    infoButton.setBitmaps(touchgfx::Bitmap(BITMAP_ICON_INFO_ID), touchgfx::Bitmap(BITMAP_ICON_INFO_ID));
    add(infoButton);

    chromArtButton.setXY(645, -1);
    chromArtButton.setBitmaps(touchgfx::Bitmap(BITMAP_CHROM_ART_OFF_BUTTON_ID), touchgfx::Bitmap(BITMAP_CHROM_ART_ON_BUTTON_ID));
    chromArtButton.forceState(true);
    chromArtButton.setAction(buttonCallback);
    add(chromArtButton);
}

TopBarContainerBase::~TopBarContainerBase()
{

}

void TopBarContainerBase::initialize()
{

}

void TopBarContainerBase::buttonCallbackHandler(const touchgfx::AbstractButton& src)
{
    if (&src == &chromArtButton)
    {
        //ChromARTButtonInteraction
        //When chromArtButton clicked emit chromArtPressed callback
        //Emit callback
        emitChromArtPressedCallback(chromArtButton.getState());
    }
}