/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "figure.h"

#include "../bnn/src/brain/config.h"

figure::~figure()
{
    //scnMgr->destroyEntity(ent);
}

figure::figure(Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal mass)
    : world(world), space(space)
{
    this->scnMgr = scnMgr;
    node = scnMgr->getRootSceneNode()->createChildSceneNode();
    body = dBodyCreate(world);
    this->mass.mass = mass;
}

void figure::step()
{
    auto *pos = dBodyGetPosition(body);
    auto *qrot = dBodyGetQuaternion(body);

    //    float x = pos[0];
    //    float y = pos[1];
    //    float z = pos[2];

    //    logging("x=" + std::to_string(x) + " y=" + std::to_string(y) + " z=" + std::to_string(z))

    node->setPosition(pos[0], pos[1], pos[2]);
    node->setOrientation(qrot[0], qrot[1], qrot[2], qrot[3]);
}

void figure::set_material(MaterialPtr materialPtr)
{
    ent->setMaterial(materialPtr);
}

static int count_name = 5;

union conv
{
    uint32 ui32;
    uint8 ui8[4];
};

/**
  @param color0, color1 - BGRA
 */
MaterialPtr figure::create_material_chess(int size, int step, uint32 color0, uint32 color1)
{
    // Create the texture
    TexturePtr texture = TextureManager::getSingleton().createManual(
                "DynamicTexture" + to_string(count_name), // name
                ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                TEX_TYPE_2D,      // type
                size, size,         // width & height
                0,                // number of mipmaps
                PF_BYTE_BGRA,     // pixel format
                TU_DEFAULT);      // usage; should be TU_DYNAMIC_WRITE_ONLY_DISCARDABLE for
    // textures updated very often (e.g. each frame)

    // Get the pixel buffer
    HardwarePixelBufferSharedPtr pixelBuffer = texture->getBuffer();

    // Lock the pixel buffer and get a pixel box
    pixelBuffer->lock(HardwareBuffer::HBL_NORMAL); // for best performance use HBL_DISCARD!
    const PixelBox& pixelBox = pixelBuffer->getCurrentLock();

    uint8* pDest = static_cast<uint8*>(pixelBox.data);

    // Fill in some pixel data. This will give a semi-transparent blue,
    // but this is of course dependent on the chosen pixel format.

    uint32 g1 = color0;
    uint32 g2 = color1;
    uint32 current = g1;

    conv converter_;

    for (size_t i = 0; i < size; i++)
    {
        for (size_t j = 0; j < size; j++)
        {
            if((j / step + i / step) % 2 == 0)
                current = g2;
            else
                current = g1;

            converter_.ui32 = current;

            *pDest++ = converter_.ui8[3]; // B
            *pDest++ = converter_.ui8[2]; // G
            *pDest++ = converter_.ui8[1]; // R
            *pDest++ = converter_.ui8[0]; // A
        }

        pDest += pixelBox.getRowSkip() * Ogre::PixelUtil::getNumElemBytes(pixelBox.format);
    }


    // Unlock the pixel buffer
    pixelBuffer->unlock();

    // Create a material using the texture
    MaterialPtr material = MaterialManager::getSingleton().create(
                "DynamicTextureMaterial" + to_string(count_name), // name
                ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    material->getTechnique(0)->getPass(0)->createTextureUnitState("DynamicTexture" + to_string(count_name));
    material->getTechnique(0)->getPass(0)->setSceneBlending(SBT_TRANSPARENT_ALPHA);

    count_name++;

    return material;

}

MaterialPtr figure::create_material_body_sign(size_t size)
{
    uint32 color_white = 0xffffffff;
    uint32 color_black = 0x000000ff;
    uint32 color_yellow = 0x00ffffff;
    uint32 color_transparent = 0x00000000;

    vector<vector<uint32>> v(size, vector<uint32>(size));

    for(auto& y : v)
        for(auto& x : y)
            x = color_transparent;

    auto circle = [&](uint32 color, int center_x, int center_y, int radius)
    {
        for(int y = -radius; y < radius; y++)
            for(int x = -radius; x < radius; x++)
                if(x * x + y * y <= radius * radius)
                    v[center_y + y][center_x + x] = color;
    };

    auto line = [&](uint32 color, int x0, int y0, int x1, int y1, int half_thickness)
    {
        Ogre::Vector3 v1(0, 0, 1);
        Ogre::Vector3 v2(x1 - x0, y1 - y0, 0);
        v2.normalise();
        Ogre::Vector3 v3 = v1.crossProduct(v2);
        v3 *= half_thickness;

        std::vector<Ogre::Vector3> absolut
        {
            Ogre::Vector3(x0, y0, 0) + v3,
                    Ogre::Vector3(x0, y0, 0) - v3,
                    Ogre::Vector3(x1, y1, 0) - v3,
                    Ogre::Vector3(x1, y1, 0) + v3
        };

        std::vector<Ogre::Vector3> relative
        {
            absolut[1] - absolut[0],
                    absolut[2] - absolut[1],
                    absolut[3] - absolut[2],
                    absolut[0] - absolut[3]
        };

        for(size_t y = 0; y < size; y++)
            for(size_t x = 0; x < size; x++)
            {
                bool b = true;

                for(size_t i = 0; i < relative.size(); i++)
                    if((Ogre::Vector3(x, y, 0) - absolut[i]).crossProduct(relative[i]).z >= 0)
                        b = false;

                if(b)
                    v[y][x] = color;
            }
    };

    auto triangle = [&](uint32 color, int x0, int y0, int x1, int y1, int x2, int y2)
    {
        std::vector<Ogre::Vector3> absolut
        {
            Ogre::Vector3(x0, y0, 0),
                    Ogre::Vector3(x1, y1, 0),
                    Ogre::Vector3(x2, y2, 0)
        };

        std::vector<Ogre::Vector3> relative
        {
            absolut[1] - absolut[0],
                    absolut[2] - absolut[1],
                    absolut[0] - absolut[2]
        };

        for(size_t y = 0; y < size; y++)
            for(size_t x = 0; x < size; x++)
            {
                bool b = true;

                for(size_t i = 0; i < relative.size(); i++)
                    if((Ogre::Vector3(x, y, 0) - absolut[i]).crossProduct(relative[i]).z <= 0)
                        b = false;

                if(b)
                    v[y][x] = color;
            }
    };

    int radius = size * 3 / 8;
    size_t center_x = size / 2;
    size_t center_y = size / 2;
    int top_x = center_x + radius * cos(90 * M_PI / 180);
    int top_y = center_y - radius * sin(90 * M_PI / 180);
    int left_x = center_x + radius * cos(210 * M_PI / 180);
    int left_y = center_y - radius * sin(210 * M_PI / 180);
    int right_x = center_x + radius * cos(330 * M_PI / 180);
    int right_y = center_y - radius * sin(330 * M_PI / 180);

    int temp = radius / 8;
    circle(color_white, top_x, top_y, temp);
    line(color_white, top_x, top_y, left_x, left_y, temp);
    circle(color_white, left_x, left_y, temp);
    line(color_white, left_x, left_y, right_x, right_y, temp);
    circle(color_white, right_x, right_y, temp);
    line(color_white, right_x, right_y, top_x, top_y, temp);

    temp = radius / 9;
    circle(color_black, top_x, top_y, temp);
    circle(color_black, left_x, left_y, temp);
    circle(color_black, right_x, right_y, temp);
    line(color_black, top_x, top_y, left_x, left_y, temp);
    line(color_black, left_x, left_y, right_x, right_y, temp);
    line(color_black, right_x, right_y, top_x, top_y, temp);

    triangle(color_yellow, top_x, top_y, left_x, left_y, right_x, right_y);


    temp = radius / 9;
    circle(color_black, center_x, center_y, temp);

    int start = 2;
    int end = 4;

    int x = temp * cos(150 * M_PI / 180);
    int y = temp * sin(150 * M_PI / 180);
    line(color_black,
              center_x + x * start,
              center_y - y * start,
              center_x + x * end,
              center_y - y * end,
              temp);

    x = temp * cos(30 * M_PI / 180);
    y = temp * sin(30 * M_PI / 180);
    line(color_black,
              center_x + x * start,
              center_y - y * start,
              center_x + x * end,
              center_y - y * end,
              temp);

    x = temp * cos(270 * M_PI / 180);
    y = temp * sin(270 * M_PI / 180);
    line(color_black,
              center_x + x * start,
              center_y - y * start,
              center_x + x * end,
              center_y - y * end,
              temp);

    // Create the texture
    TexturePtr texture = TextureManager::getSingleton().createManual(
                "DynamicTexture" + to_string(count_name), // name
                ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                TEX_TYPE_2D,      // type
                size, size,         // width & height
                0,                // number of mipmaps
                PF_BYTE_BGRA,     // pixel format
                TU_DEFAULT);      // usage; should be TU_DYNAMIC_WRITE_ONLY_DISCARDABLE for
    // textures updated very often (e.g. each frame)

    // Get the pixel buffer
    HardwarePixelBufferSharedPtr pixelBuffer = texture->getBuffer();

    // Lock the pixel buffer and get a pixel box
    pixelBuffer->lock(HardwareBuffer::HBL_NORMAL); // for best performance use HBL_DISCARD!
    const PixelBox& pixelBox = pixelBuffer->getCurrentLock();

    conv converter_;

    {
        uint8* pDest = static_cast<uint8*>(pixelBox.data);

        for (size_t y = 0; y < size; y++)
        {
            for (size_t x = 0; x < size; x++)
            {
                converter_.ui32 = v[y][x];

                *pDest++ = converter_.ui8[3]; // B
                *pDest++ = converter_.ui8[2]; // G
                *pDest++ = converter_.ui8[1]; // R
                *pDest++ = converter_.ui8[0]; // A
            }

            pDest += pixelBox.getRowSkip() * Ogre::PixelUtil::getNumElemBytes(pixelBox.format);
        }
    }

    // Unlock the pixel buffer
    pixelBuffer->unlock();

    // Create a material using the texture
    MaterialPtr material = MaterialManager::getSingleton().create(
                "DynamicTextureMaterial" + to_string(count_name), // name
                ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    material->getTechnique(0)->getPass(0)->createTextureUnitState("DynamicTexture" + to_string(count_name));
    material->getTechnique(0)->getPass(0)->setSceneBlending(SBT_TRANSPARENT_ALPHA);

    count_name++;

    return material;

}
