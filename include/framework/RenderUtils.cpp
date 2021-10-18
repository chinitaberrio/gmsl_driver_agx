#include "RenderUtils.hpp"

namespace renderutils
{
void renderFPS(dwRenderEngineHandle_t renderEngine, const float32_t fps)
{
    if (renderEngine != DW_NULL_HANDLE)
    {
        CHECK_DW_ERROR(dwRenderEngine_setColor({1.0f, 1.0f, 1.0f, 1.0f}, renderEngine));
        CHECK_DW_ERROR(dwRenderEngine_setFont(DW_RENDER_ENGINE_FONT_VERDANA_12, renderEngine));

        // render
        constexpr uint32_t textBufferSize = 128;
        char fpsText[textBufferSize];
        snprintf(fpsText, textBufferSize, "FPS: %.02f", fps);
        dwVector2f range{.x = 1.0f, .y = 1.0f};
        dwVector2f fpsTextPos = {.x = 0.f, .y = 1.f};

        renderText(fpsText, fpsTextPos, range, renderEngine);
    }
}

void renderText(const char* textBuff,
                const dwVector2f& textPos,
                const dwVector2f& range,
                dwRenderEngineHandle_t renderEngine)
{
    // store previous tile
    uint32_t previousTile = 0;
    CHECK_DW_ERROR(dwRenderEngine_getTile(&previousTile, renderEngine));

    // select default tile
    CHECK_DW_ERROR(dwRenderEngine_setTile(0, renderEngine));

    // get default tile state
    dwRenderEngineTileState previousDefaultState{};
    CHECK_DW_ERROR(dwRenderEngine_getState(&previousDefaultState, renderEngine));

    // set text render settings
    CHECK_DW_ERROR(dwRenderEngine_setModelView(&DW_IDENTITY_MATRIX4F, renderEngine));
    CHECK_DW_ERROR(dwRenderEngine_setCoordinateRange2D(range, renderEngine));

    // render
    dwRenderEngine_renderText2D(textBuff, textPos, renderEngine);

    // restore previous settings
    CHECK_DW_ERROR(dwRenderEngine_setState(&previousDefaultState, renderEngine));
    CHECK_DW_ERROR(dwRenderEngine_setTile(previousTile, renderEngine));
}
}
