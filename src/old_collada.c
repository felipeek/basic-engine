#include "collada.h"
#include <string.h>
#include <libxml/parser.h>
#include "graphics.h"

typedef struct {
	xmlDoc* document;
} ColladaInformation;

static xmlNode* getMaterialNode(xmlNode* root)
{
	xmlNode* node = NULL;
    xmlNode* firstChild = NULL;
	xmlNode* libraryGeometriesNode = NULL;
	xmlNode* geometryNode = NULL;
	xmlNode* meshNode = NULL;
	xmlNode* materialNode = NULL;
	
	firstChild = root->children;
    for (node = firstChild; node; node = node->next) {
		if (!strcmp(node->name, "library_geometries")) {
			libraryGeometriesNode = node;
			break;
		}
    }

	// We only select the first geometry found
	firstChild = libraryGeometriesNode->children;
    for (node = firstChild; node; node = node->next) {
		if (!strcmp(node->name, "geometry")) {
			geometryNode = node;
			break;
		}
    }

	firstChild = geometryNode->children;
    for (node = firstChild; node; node = node->next) {
		if (!strcmp(node->name, "mesh")) {
			meshNode = node;
			break;
		}
    }

    firstChild = meshNode->children;
    for (node = firstChild; node; node = node->next) {
		if (!strcmp(node->name, "polylist")) {
			xmlAttr* firstProperty = node->properties;
			xmlAttr* attrNode;
			xmlChar* attrContent;

			for (attrNode = firstProperty; attrNode; attrNode = attrNode->next) {
				if (!strcmp(attrNode->name, "material")) {
					materialNode = node;
					break;
				}
			}

			if (materialNode != NULL)
				break;
		}
    }

	return materialNode;
}

VertexWithJoints* verticesGetFromCollada(void* handler)
{
	xmlNode* node = NULL;
    xmlNode* firstChild = NULL;

	ColladaInformation* ci = handler;
    xmlNode* root = xmlDocGetRootElement(ci->document);

	//const s8* materialName = getMaterialName(root);
	xmlNode* materialNode = getMaterialNode(root);

	xmlChar* 

	firstChild = materialNode->children;
    for (node = firstChild; node; node = node->next) {
		if (!strcmp(node->name, "input")) {
			xmlAttr* firstProperty = node->properties;
			xmlAttr* attrNode;
			xmlChar* attrContent;
			xmlAttr* semanticAttr;

			for (attrNode = firstProperty; attrNode; attrNode = attrNode->next) {
				if (!strcmp(attrNode->name, "semantic")) {
					semanticAttr = attrNode;
					break;
				}
			}

			if (semanticAttr == NULL)
				continue;

			xmlChar* semanticValue = xmlNodeGetContent(semanticAttr);

			if (!strcmp(semanticValue, "VERTEX")) {

			} else if (!strcmp(semanticValue, "NORMAL")) {

			} else if (!strcmp(semanticValue, "TEXCOORD")) {

			} else if (!strcmp(semanticValue, "COLOR")) {

			}
			break;
		}
    }
}

void* colladaInit(const s8* colladaFile)
{
	xmlKeepBlanksDefault(0);
    xmlDoc* document = xmlReadFile(colladaFile, NULL, 0);

	ColladaInformation* ci = malloc(sizeof(ColladaInformation));
	ci->document = document;

	verticesGetFromCollada(ci);

    return ci;
}

s32 colladaDestroy(void* handler)
{
	ColladaInformation* ci = (ColladaInformation*)handler;
	xmlFreeDoc(ci->document);
}