
"use strict";

let YAMLExport = require('./YAMLExport.js')
let YAMLImport = require('./YAMLImport.js')
let SaveMap = require('./SaveMap.js')
let DeleteMap = require('./DeleteMap.js')
let DeleteAnnotations = require('./DeleteAnnotations.js')
let GetAnnotationsData = require('./GetAnnotationsData.js')
let SaveAnnotationsData = require('./SaveAnnotationsData.js')
let GetAnnotations = require('./GetAnnotations.js')
let SetRelationship = require('./SetRelationship.js')
let PubAnnotationsData = require('./PubAnnotationsData.js')
let ListWorlds = require('./ListWorlds.js')
let EditAnnotationsData = require('./EditAnnotationsData.js')
let RenameMap = require('./RenameMap.js')
let ResetDatabase = require('./ResetDatabase.js')
let ListMaps = require('./ListMaps.js')
let SetKeyword = require('./SetKeyword.js')
let PublishMap = require('./PublishMap.js')

module.exports = {
  YAMLExport: YAMLExport,
  YAMLImport: YAMLImport,
  SaveMap: SaveMap,
  DeleteMap: DeleteMap,
  DeleteAnnotations: DeleteAnnotations,
  GetAnnotationsData: GetAnnotationsData,
  SaveAnnotationsData: SaveAnnotationsData,
  GetAnnotations: GetAnnotations,
  SetRelationship: SetRelationship,
  PubAnnotationsData: PubAnnotationsData,
  ListWorlds: ListWorlds,
  EditAnnotationsData: EditAnnotationsData,
  RenameMap: RenameMap,
  ResetDatabase: ResetDatabase,
  ListMaps: ListMaps,
  SetKeyword: SetKeyword,
  PublishMap: PublishMap,
};
