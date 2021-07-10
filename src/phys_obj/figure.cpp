/*
 *   device_3d
 *   created by Ilya Shishkin
 *   cortl@8iter.ru
 *   https://github.com/cortl0/device_3d
 *   licensed by GPL v3.0
 */

#include "figure.h"

figure::figure(Ogre::SceneManager* scnMgr, dWorldID world, dSpaceID space, dReal mass) : world(world), space(space)
{
    this->scnMgr = scnMgr;
    node = scnMgr->getRootSceneNode()->createChildSceneNode();
    body = dBodyCreate(world);
    this->mass.mass = mass;
}

void figure::step()
{
    auto pos = dBodyGetPosition(body);
    auto qrot = dBodyGetQuaternion(body);

    //node->setPosition(pos[1], pos[2], pos[0]);

    node->setPosition(pos[0], pos[1], pos[2]);
    node->setOrientation(qrot[0], qrot[1], qrot[2], qrot[3]);
}

figure::~figure()
{
    //scnMgr->destroyEntity(ent);
}
static int count_name = 5;
MaterialPtr figure::create_material(int size, int step, uint8 color1, uint8 color2, uint8 a)
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

    uint8 g1 = color1;
    uint8 g2 = color2;
    uint8 current = g1;

    for (size_t i = 0; i < size; i++)
    {
        for (size_t j = 0; j < size; j++)
        {
            if((j / step + i / step) % 2 == 0)
                current = g2;
            else
                current = g1;

            *pDest++ = current; // B
            *pDest++ = current; // G
            *pDest++ = current; // R
            *pDest++ = a; // A
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
